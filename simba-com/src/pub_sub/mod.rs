mod broker;
mod channel;
mod client;
mod multi_client;

pub use broker::Broker;
pub use broker::GenericBroker;
pub use broker::PathKey;
pub use channel::Channel;
pub use client::Client;
pub use multi_client::MultiClient;

#[cfg(test)]
mod tests {
    use std::{sync::{Arc, Barrier, Mutex}, thread};

    use crate::pub_sub::Broker;

    #[test]
    fn subscribe_after_send() {
        let broker = Arc::new(Mutex::new(Broker::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel::<i32>(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));
        
        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            client.send(3, (), 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            barrier_clone.wait();
            barrier_clone.wait();
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            let message = client.try_receive(&(), 3.2);
            assert_eq!(message, None);
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages();
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn send_message_simple() {
        let broker = Arc::new(Mutex::new(Broker::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel::<i32>(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));
        
        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            client.send(3, (), 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(&(), 3.2);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages();
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
    }

    #[test]
    fn late_reception() {
        let broker = Arc::new(Mutex::new(Broker::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel::<i32>(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));
        
        let barrier_clone = barrier.clone();

        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            client.send(3, (), 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(&(), 3.1);
            assert!(message.is_none());
            let message = client.try_receive(&(), 3.2);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages();
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

        broker.lock().unwrap().add_channel_conditionnal::<i32, i32>(channel_name.to_string(), |arg1: i32, arg2: i32| arg1 == arg2);

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel_conditionnal::<i32, i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            client.send(3, 5, 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel_conditionnal::<i32, i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.0);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(&3, 3.2);
            assert!(message.is_none());
            let message = client.try_receive(&5, 3.2);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages();
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();

    }

    #[test]
    fn reception_delay() {
        let broker = Arc::new(Mutex::new(Broker::new(0.1)));

        let channel_name = "hello";

        broker.lock().unwrap().add_channel::<i32>(channel_name.to_string());

        let barrier = Arc::new(Barrier::new(3));
        
        let barrier_clone = barrier.clone();

        let broker_clone = broker.clone();
        let handle1 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.);
            client.send(3, (), 3.2);
            barrier_clone.wait();
            barrier_clone.wait();
        });

        let barrier_clone = barrier.clone();
        let broker_clone = broker.clone();
        let handle2 = thread::spawn(move || {
            let mut channel = broker_clone.lock().unwrap().get_channel::<i32>(&channel_name.to_string()).unwrap();
            let client = channel.client(0.5);
            barrier_clone.wait();
            barrier_clone.wait();
            let message = client.try_receive(&(), 3.2);
            assert!(message.is_none());
            let message = client.try_receive(&(), 3.3);
            assert!(message.is_none());
            let message = client.try_receive(&(), 3.8);
            assert_eq!(message, Some(3));
        });

        let handle3 = thread::spawn(move || {
            barrier.wait();
            broker.lock().unwrap().process_messages();
            barrier.wait();
        });

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();

    }
}
