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
    signal: Receiver<Signal>,
    signal_source: Sender<Signal>,
}

pub struct Signal {}

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
    topic: String,
    reciever: Receiver<Arc<dyn Any + Send + Sync + 'static>>,
    _phantom: PhantomData<T>,
}

impl<T: Any + Send + Sync + 'static> Subscription<T> {
    /// Tries to receive a value from the subscribed topic, but will not block if no data is available.
    pub fn try_recv(&mut self) -> Option<Arc<T>> {
        match self.reciever.try_recv() {
            Ok(value) => Some(
                value
                    .downcast::<T>()
                    .expect("Received value was not of the expected type"),
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
    /// Receives a value from the subscribed topic, and will block if no data is available.
    pub fn recv(&mut self) -> Arc<T> {
        self.reciever
            .recv()
            .expect("Other end of channel was unexpectedly closed")
            .downcast::<T>()
            .expect("Received value was not of the expected type")
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }
}

#[derive(Clone)]
pub struct Publisher<T: Any + Send + Sync + 'static> {
    topic: String,
    send: Sender<Arc<dyn Any + Send + Sync + 'static>>,
    signal: Sender<Signal>,
    _p: PhantomData<T>,
}

impl<T: Any + Send + Sync + 'static> Publisher<T> {
    /// Publishes a value wrapped in an `Arc` to the topic.
    pub fn publish(&mut self, value: Arc<T>) {
        // if the other end is closed or there was an error, ignore
        _ = self.send.send(value);
        _ = self.signal.send(Signal {});
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }
}

impl PubSub {
    pub fn new() -> Self {
        let (send, receive) = channel();
        Self {
            topics: HashMap::new(),
            signal: receive,
            signal_source: send,
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
            topic: topic.to_string(),
            send: t.incoming_sender.clone(),
            signal: self.signal_source.clone(),
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
            topic: topic.to_owned(),
            reciever: recv,
            _phantom: PhantomData,
        }
    }

    /// Proceses and distributes messages to all subscribers.
    pub fn tick(&mut self) {
        for (_topic, t) in self.topics.iter_mut() {
            // read all the incoming messages and distribute them by cloning the Arc's

            while let Ok(v) = t.incoming_recv.try_recv() {
                // iterate over all outgoing, dropping any chanels that have been disconnected
                t.outgoing.retain_mut(|s| s.send(v.clone()).is_ok());
            }

            // empty all signals as well
        }
        while self.signal.try_recv().is_ok() {}
    }

    /// Creates a ticker that calls tick() continously when updated.
    /// On desktop this spawns a background thread, on wasm32 it runs the tick
    /// method directly on the main thread.
    pub fn to_ticker(self, waker: impl FnMut() + Send + 'static) -> ticker::PubSubTicker {
        ticker::PubSubTicker::new(self, waker)
    }
}

impl Default for PubSub {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(target_arch = "wasm32")]
pub mod ticker {
    use crate::PubSub;

    pub struct PubSubTicker {
        pubsub: PubSub,
    }

    impl PubSubTicker {
        pub fn new(pubsub: PubSub, _waker: impl FnMut() + Send + 'static) -> Self {
            // waker is intentionally unused since we will not "wake up" to do
            // repaint on publish either way
            Self { pubsub }
        }

        pub fn tick(&mut self) {
            self.pubsub.tick()
        }
        pub fn stop(self) {}
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub mod ticker {
    use crate::PubSub;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::thread::{self, JoinHandle};
    use std::time::Duration;

    pub struct PubSubTicker {
        thread_handle: PubSubThreadHandle,
    }

    impl PubSubTicker {
        pub fn new(pubsub: PubSub, waker: impl FnMut() + Send + 'static) -> Self {
            Self {
                thread_handle: PubSubThreadHandle::new(pubsub, waker),
            }
        }

        pub fn tick(&mut self) {
            // do nothing on desktop
        }

        pub fn stop(self) {
            self.thread_handle.stop();
        }
    }

    pub struct PubSubThreadHandle {
        handle: JoinHandle<anyhow::Result<()>>,
        running: Arc<AtomicBool>,
    }

    impl PubSubThreadHandle {
        fn new(pubsub: PubSub, waker: impl FnMut() + Send + 'static) -> Self {
            let running = Arc::new(AtomicBool::new(true));

            let handle = thread::spawn({
                let running = running.clone();
                move || Self::tick_thread(pubsub, running, waker)
            });

            Self { handle, running }
        }

        pub fn stop(self) {
            self.running.store(false, Ordering::Relaxed);
            self.handle.join().unwrap().unwrap();
        }

        fn tick_thread(
            mut pubsub: PubSub,
            running: Arc<AtomicBool>,
            mut waker: impl FnMut() + Send + 'static,
        ) -> anyhow::Result<()> {
            'outer: loop {
                // block on the signal

                loop {
                    let result = pubsub.signal.recv_timeout(Duration::from_millis(500));
                    if !running.load(Ordering::Relaxed) {
                        println!("Stopping Tick Thread");
                        break 'outer;
                    }

                    match result {
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                        Err(e) => return Err(e.into()),
                        _ => break,
                    };
                }

                // process messages
                pubsub.tick();

                // call the waker to notify anyone listening about the newly available messages
                waker();
            }

            Ok(())
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
