use std::{collections::VecDeque, sync::Arc};
use iced::{Application, Subscription, Task, widget::{button, column, text}, window::{self, Id, Settings, events}};
use robomorph::{com_channels::{ChannelConfig, UDPChannel}, events_management::Event, process::{ModuleLinker, Worker, WorkerFactory}};
use iced::window::Event as WindowEvent;


fn main() -> iced::Result {
    let udp=Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                                ModuleLinker::new("UDP1_WORKER".to_string())), 
                                                                                8090, "127.0.0.1".to_string(), 8080, 50));
    //Run the application
    //  update()            => Fondamental function to execute callbacks to gui component events
    //  view()              => Fondamental function to display the GUI components on a window
    //      .subscription() => Function to allow the software to subscribe to general window event like opening, closing window, focus, unfocus, etc...
    //      .run_with()     => Function to run the application with a initialization function and task.
    return iced::application("Controller", Controller::update, Controller::view)
        .subscription(|state: &Controller| state.subscription())
        .run_with( || (Controller::new(udp), Task::none()));;
}

#[derive(Debug, Clone)]
enum ControllerEvent {
    ButtonPressed,
    FrameReceived,
    ImageReceived,
    OSEvent(u32),
    NoImportant,
}

#[derive(Default)]
struct Controller {
    //event_queue: VecDeque<ControllerEvent>,
    worker_factory: WorkerFactory
}

impl Controller {
    fn new(udp: Arc<UDPChannel>) -> Controller {
        let mut worker_factory= WorkerFactory::new(vec![Arc::new(Worker::new("UDP WORKER".to_string(), udp.clone(), udp.clone().frequency))]);
        //worker_factory.register_process("UDP WORKER".to_string(), udp.clone(), udp.clone().frequency);
        return Controller { worker_factory };
        //events().
        //self.worker_factory.register_process("UDP WORKER", udp, 50);
    }
    fn update(&mut self, event: ControllerEvent) {
        match event {
            ControllerEvent::ButtonPressed => {
            },
            ControllerEvent::FrameReceived => {},
            ControllerEvent::ImageReceived => {},
            ControllerEvent::OSEvent(ev_id) => {
                if ev_id == 100 {
                    self.worker_factory.start_all_process_workers();
                }
                else if ev_id == 200 {
                    self.worker_factory.end_all_process_workers();
                }
            },
            _ => {

            }
        }
    }

    fn view(&self) -> iced::Element<ControllerEvent> {
        column![
            text("Salut"),
            button("Increase").on_press(ControllerEvent::ButtonPressed),
        ]
        .into()
    }

    fn subscription(&self) -> Subscription<ControllerEvent> {
        window::events().map(|(id, ev)| {
            match ev {
                WindowEvent::Opened { position, size } => {
                    return ControllerEvent::OSEvent(100);
                },
                WindowEvent::Closed => {
                    return ControllerEvent::OSEvent(200);
                },
                _ => {
                    return ControllerEvent::NoImportant;
                }
            }
        })
    }
}