use std::{
    any::{type_name, Any, TypeId},
    collections::HashMap,
    marker::PhantomData,
    sync::{
        mpsc::{self, channel, Receiver, Sender},
        Arc,
    },
};

/// A simple publish/subscribe system that allows sending and subscribing to values on different topics.
/// Each topic name is allocated to a single type, attempts to subscribe and publish to the same topic with
/// different types will panic!
pub struct PubSub {
    topics: HashMap<String, Topic>,
}

struct Topic {
    value_type: TypeId,
    value_name: &'static str,
    incoming_sender: Sender<Arc<dyn Any + Send + Sync + 'static>>,
    incoming_recv: Receiver<Arc<dyn Any + Send + Sync + 'static>>,
    outgoing: Vec<Sender<Arc<dyn Any + Send + Sync + 'static>>>,
}

impl Topic {
    fn new<T: Any + Send + Sync + 'static>() -> Self {
        // create the channel where items will be sent to when published
        let (send, recv) = channel();

        Self {
            value_type: TypeId::of::<T>(),
            value_name: type_name::<T>(),
            incoming_sender: send,
            incoming_recv: recv,
            outgoing: Vec::new(),
        }
    }
}

pub struct Subscription<T: Any + Send + Sync + 'static> {
    reciever: Receiver<Arc<dyn Any + Send + Sync + 'static>>,
    _phantom: PhantomData<T>,
}

impl<T: Any + Send + Sync + 'static> Subscription<T> {
    /// Tries to recieve a value from the subscribed topic.
    pub fn try_recv(&mut self) -> Option<Arc<T>> {
        match self.reciever.try_recv() {
            Ok(value) => Some(
                value
                    .downcast::<T>()
                    .expect("Recieved value was not of the expected type"),
            ),
            Err(e) => {
                match e {
                    mpsc::TryRecvError::Empty => {}
                    mpsc::TryRecvError::Disconnected => {
                        println!("Disconnected!")
                    }
                }
                None
            }
        }
    }
}

pub struct Publisher<T: Any + Send + Sync + 'static> {
    send: Sender<Arc<dyn Any + Send + Sync + 'static>>,
    _p: PhantomData<T>,
}

impl<T: Any + Send + Sync + 'static> Publisher<T> {
    /// Publishes a value wrapped in an `Arc` to the topic.
    pub fn publish(&mut self, value: Arc<T>) {
        self.send.send(value).unwrap();
    }
}

impl PubSub {
    pub fn new() -> Self {
        Self {
            topics: HashMap::new(),
        }
    }

    fn get_topic_by_name_or_insert<T: Any + Send + Sync + 'static>(
        &mut self,
        topic: &str,
    ) -> &mut Topic {
        let t = self.topics.entry(topic.into()).or_insert(Topic::new::<T>());

        // make sure this topic was not previously claimed with a different type.
        assert!(
            t.value_type == TypeId::of::<T>(),
            "Topic {topic} already claimed by type '{}', but current type is '{}'",
            t.value_name,
            type_name::<T>()
        );

        t
    }

    /// Register as a publisher of the specific type to the topic name. Panics if the topic has already been allocated to values of a different type.
    pub fn publish<T: Any + Send + Sync + 'static>(&mut self, topic: &str) -> Publisher<T> {
        let t = self.get_topic_by_name_or_insert::<T>(topic);

        Publisher {
            send: t.incoming_sender.clone(),
            _p: PhantomData,
        }
    }

    /// Subscribe to messages of the specific type on the topic name. Panics if the topic has already been allocated to values of a different type.
    pub fn subscribe<T: Any + Send + Sync + 'static>(&mut self, topic: &str) -> Subscription<T> {
        let t = self.get_topic_by_name_or_insert::<T>(topic);

        // create a channel for receiving the published messages
        let (send, recv) = channel();

        t.outgoing.push(send);

        Subscription {
            reciever: recv,
            _phantom: PhantomData,
        }
    }

    /// Proceses and distributes messages to all subscribers.
    pub fn tick(&mut self) {
        for (_topic, t) in self.topics.iter() {
            // read all the incoming messages and distribute them by cloning the Arc's

            while let Ok(v) = t.incoming_recv.try_recv() {
                for s in t.outgoing.iter() {
                    // TODO: handle closing the subsciptions and thus the channel itself.
                    s.send(v.clone()).unwrap();
                }
            }
        }
    }
}

// #[derive(Debug)]
// struct Data {
//     d: Vec<u32>,
// }

// pub fn test() {
//     // test pubsub quickly
//     let mut ps = PubSub::new();
//     let mut s = ps.subscribe::<Data>("test");
//     let mut s2 = ps.subscribe::<Data>("test");
//     let mut p = ps.publish::<Data>("test");

//     p.publish(Data {
//         d: vec![1, 2, 3, 4],
//     });

//     ps.tick();

//     if let Some(d) = s.try_recv() {
//         dbg!(&d);
//         println!("{d:p}");
//     }

//     if let Some(d2) = s2.try_recv() {
//         dbg!(&d2);
//         println!("{d2:p}");
//     }
// }
