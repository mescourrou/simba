# Message Handler

The message handler implementation is very simple. You only need to implement the `MessageHandler` trait:
```Rust
fn handle_message(&mut self, robot: &mut Node, from: &String, message: &Value, time: f32) -> Result<(), ()>;
```

The function should return Ok if the message was treated, otherwise Err. If the message is treated, no other MessageHandler is called.

The MessageHandler trait can be implemented by another Module, the PluginAPI implementation should be adapted to return a same instance for both calls.


## Code template

```Rust
#[derive(Debug)]
struct MyWonderfulMessageHandler {}

#[derive(Debug, Serialize, Deserialize)]
struct MyMessage {}

impl MessageHandler for MyWonderfulMessageHandler {
    fn handle_message(
        &mut self,
        robot: &mut simba::node::Node,
        from: &String,
        message: &serde_json::Value,
        time: f32,
    ) -> Result<(), ()> {
        
        match serde_json::from_value::<MyMessage>(message.clone()) {
            Err(e) => Err(()),
            Ok(m) => {
                println!("Receive message {}", message);
                Ok(())
            }
        }
    }
}
```