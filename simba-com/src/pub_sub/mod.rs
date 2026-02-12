mod broker;
mod channel;
mod client;
mod multi_client;

pub use broker::Broker;
pub use broker::BrokerTrait;
pub use broker::BrokerTraitExtended;
pub use broker::BrokerTraitProcessing;
pub use broker::PathBroker;
pub use broker::PathKey;
pub use channel::Channel;
pub use client::Client;
pub use multi_client::MultiClient;
pub use multi_client::MultiClientTrait;
pub use multi_client::PathMultiClient;

pub type ConditionType<ConditionArgType> = fn(ConditionArgType, ConditionArgType) -> bool;

#[cfg(test)]
mod tests {
    use std::{
        collections::HashMap,
        sync::{Arc, Barrier, Mutex},
        thread,
    };

    use crate::pub_sub::{
        Broker, BrokerTrait, BrokerTraitExtended, PathBroker, PathKey,
        broker::BrokerTraitProcessing,
    };

    #[test]
    fn subscribe_after_send() {
        let broker = Arc::new(Mutex::new(Broker::<String, i32, String, ()>::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread1".to_string(), 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            barrier_clone.wait();
            barrier_clone.wait();
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.0);
            let message = client.try_receive(3.2);
            assert_eq!(message, None);
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn subscribe_before_creation() {
        let broker = Arc::new(Mutex::new(Broker::<String, i32, String, u8>::new(0.1)));

        let channel_name = "hello";

        let barrier = Arc::new(Barrier::new(3));

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            assert!(
                broker_clone
                    .lock()
                    .unwrap()
                    .get_channel(&channel_name.to_string())
                    .is_none()
            );
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            barrier_clone.wait();
            barrier_clone.wait();
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.0);
            let message = client.try_receive(3.2);
            assert_eq!(message, None);
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker
                .lock()
                .unwrap()
                .add_channel_conditionnal(channel_name.to_string(), |arg1: u8, arg2: u8| {
                    arg1 == arg2
                });
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn send_message_simple() {
        let broker = Arc::new(Mutex::new(Broker::<String, i32, String, u8>::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread1".to_string(), 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(3.2);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn late_reception() {
        let broker = Arc::new(Mutex::new(Broker::<String, u8, String, u8>::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));

        let barrier_clone = barrier.clone();

        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread1".to_string(), 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(3.1);
            assert!(message.is_none());
            let message = client.try_receive(3.2);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn send_message_condition() {
        let broker = Arc::new(Mutex::new(Broker::new(0.1)));

        let channel_name = "hello";

        let barrier = Arc::new(Barrier::new(3));

        broker
            .lock()
            .unwrap()
            .add_channel_conditionnal(channel_name.to_string(), |arg1: i32, arg2: i32| {
                arg1 == arg2
            });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread1".to_string(), 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();

            client.send(5, 3.5);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(3.2);
            assert!(message.is_none());
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(3.5);
            assert_eq!(message, Some(5));
        });

        let mut node_states = HashMap::new();

        // Start with different states
        node_states.insert("thread1".to_string(), 3);
        node_states.insert("thread2".to_string(), 0);

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(Some(&node_states));
            barrier.wait();
            // Change state to allow the second message to be sent
            node_states.insert("thread1".to_string(), 0);
            barrier.wait();
            broker.lock().unwrap().process_messages(Some(&node_states));
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn reception_delay() {
        let broker = Arc::new(Mutex::new(Broker::<String, u8, String, u8>::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));

        let barrier_clone = barrier.clone();

        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread1".to_string(), 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name.to_string())
                .unwrap();
            let client = channel.client("thread2".to_string(), 0.5);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(3.2);
            assert!(message.is_none());
            let message = client.try_receive(3.3);
            assert!(message.is_none());
            let message = client.try_receive(3.8);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn path_broker_meta() {
        let broker = Arc::new(Mutex::new(PathBroker::<i8, i8, u8>::new(0.1)));

        let channel_name = PathKey::new(vec!["hello".to_string(), "world".to_string()], true);

        broker
            .lock()
            .unwrap()
            .add_metachannel(channel_name.clone(), None);
        assert!(
            broker
                .lock()
                .unwrap()
                .meta_exists(&PathKey::new(vec!["hello".to_string()], true))
        );
        assert!(broker.lock().unwrap().meta_exists(&channel_name));
    }

    #[test]
    fn path_broker() {
        let broker = Arc::new(Mutex::new(PathBroker::<i8, i8, u8>::new(0.1)));

        let barrier = Arc::new(Barrier::new(3));

        let meta_channel_name = PathKey::new(vec!["hello".to_string()], true);
        let channel_name1 = PathKey::new(
            vec!["hello", "world1"]
                .iter()
                .map(|s| s.to_string())
                .collect::<Vec<String>>(),
            true,
        );
        let channel_name2 = PathKey::from_str("/hello/world2");

        broker.lock().unwrap().add_channel(channel_name1.clone());
        assert!(broker.lock().unwrap().get_channel(&channel_name1).is_some());
        assert!(broker.lock().unwrap().meta_exists(&meta_channel_name));

        broker.lock().unwrap().add_channel(channel_name2.clone());
        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name1)
                .unwrap();
            let client = channel.client(0, 0.0);
            client.send(3, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone
                .lock()
                .unwrap()
                .get_channel(&channel_name2)
                .unwrap();
            let client = channel.client(1, 0.0);
            client.send(4, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages(None);
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }
}
