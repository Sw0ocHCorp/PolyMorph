use std::{sync::{Arc, Mutex}, thread};

use plotters::{chart::ChartBuilder, prelude::{BitMapBackend, IntoDrawingArea}, series::LineSeries, style::{RED, WHITE}};
use robomorph::{events_management::{Event, Observer}, messages::Message, process::{ModuleLinker, Process, WorkerFactory}};
use slint::{Image, Model, ModelRc, Rgb8Pixel, SharedPixelBuffer, VecModel};


slint::slint!(import { ControllerWindow } from "ui/gui.slint";);


pub struct Controller {
    window: Arc<Mutex<ControllerWindow>>,
    worker_factory: Arc<Mutex<WorkerFactory>>,
    linker: Arc<Mutex<ModuleLinker>>,
}

impl Controller {
    pub fn new(factory: WorkerFactory, linker: ModuleLinker) -> Self {
        
        return Controller { window: Arc::new(Mutex::new(ControllerWindow::new().expect("ERROR: GUI creation failed"))), 
                            worker_factory: Arc::new(Mutex::new(factory)), 
                            linker:Arc::new(Mutex::new(linker)) 
                        };
    }

    fn plot_data(data: Vec<(f64, f64)>, plot_type: String, width: u32, height: u32) -> Option<SharedPixelBuffer<Rgb8Pixel>> {
        let mut plot_content= SharedPixelBuffer::new(width, height);
        let plot_display_backend= BitMapBackend::with_buffer(plot_content.make_mut_bytes(), (width, height));
        let root= plot_display_backend.into_drawing_area();
        let mut chart= ChartBuilder::on(&root).margin(10);
        match ChartBuilder::on(&root).margin(10).build_cartesian_2d(-50.0..50.0, -50.0..50.0) {
            Ok(mut chart) => {
                if let Err(_)= chart.configure_mesh().draw() {
                    println!("WARNING: Grid displaying failed");
                }
                //The data.iter().map(|(x, y)| (*x, *y)) allow to pass iter of (x,y) values and not &(x,y)
                if let Err(_) = chart.draw_series(LineSeries::new(data.iter().map(|(x, y)| (*x, *y)), &RED)) {
                    println!("WARNING: The series can't be plotted");
                }
                if let Err(_) = root.present() {
                    println!("ERROR: Unable to update the plot data");
                }
            },
            Err(_) => {
                println!("ERROR: Chart creation failed");
                return None;
            },
        }
        drop(root);
        //Return the SharedPixelBuffer to be thread safe because self is Arc<Self> (required thread safe code)
        //  /!\The conversion to Image needs to be execute in the GUI thread
        return Some(plot_content);
    }
        

    pub fn process_event_message(&mut self, event_msg: Message) {
        /*match event_msg {
            Message::Sentence(sentence) => {
                println!("Sentence= Incoming data {}", sentence);
            },
            Message::Frame(frame) => {
                if let Ok(data)= String::from_utf8(frame) {
                    println!("Frame= Incoming data {}", data);
                }
            },
            Message::Image() => todo!(),
            Message::LidarMeasurements(hash_map) => {
                let mut lidar_meas:Vec<(f64, f64)>= Vec::new();
                for (angle, dist) in hash_map {
                    lidar_meas.push((dist*f64::cos(angle.into()), dist*f64::sin(angle.into())));
                }
            },
        }*/
    }

    pub fn run(&mut self)  -> Result<(), slint::PlatformError> {
        if let Ok(mut factory) = self.worker_factory.clone().try_lock() {
            factory.start_all_process_workers();
        }
        match self.window.try_lock() {
            Ok(mut window) => {
                let plot_width= window.get_lidar_measurement().size().width.clone();
                let plot_height= window.get_lidar_measurement().size().height.clone();
                window.on_plot_data(move |points: slint::ModelRc<PlotPoint>, chart_type: slint::SharedString| {
                    let row_count = points.row_count();
                    let mut pts = Vec::with_capacity(row_count);
                    for i in 0..row_count {
                        if let Some(p) = points.row_data(i) {
                            pts.push((p.x as f64, p.y as f64));
                        }
                    }
                    if let Some(chart_content) = Controller::plot_data(pts, chart_type.to_string(), plot_width, plot_height) {
                    }
                });
                let is_running= true;
                let linkr= self.linker.clone();
                let running_thread= thread::Builder::new().name("GUI Thread".to_string())
                .spawn(move || {
                    while is_running == true {
                        if let Ok(mut linker) = linkr.clone().try_lock() {
                            if let Some(mut lnk)= linker.get_data_observer() {
                                while lnk.is_data_in_buffer() {
                                    if let Some(data) = lnk.get_incoming_data() {
                                        match data {
                                            Message::Command(frame) => {
                                                println!("Command Incoming data {} ", frame);
                                            },
                                            Message::Image() => {
                                                println!("Image Incoming data");
                                            },
                                            Message::LidarMeasurements(lidar_mes) => {
                                                println!("Incoming data LIDAR MEASUREMENTS");
                                            },
                                        }
                                    }
                                    /*
                                            Message::Image() => todo!(),
                                            Message::LidarMeasurements(hash_map) => {
                                                println!("Incoming data");
                                            },*/
                                }
                            }
                        }
                    }
                });


                return window.run();

            },
            Err(_) => {
                println!("ERROR: Controller Window is busy");
                return Ok(());
            }
        }
        
    }

    pub fn update_gui_plot(&mut self, data: Vec<(f64, f64)>, chart_type: String) {
        match self.window.try_lock() {
            Ok(window) => {
                let points: Vec<PlotPoint> = data.into_iter()
                                            .map(|(x, y)| PlotPoint { x: x as f32, y: y as f32, z: 0.0 })
                                            .collect();

                // Convert to a model:
                let model = VecModel::from(points);
                window.invoke_plot_data(ModelRc::new(model), chart_type.into());
            },
            Err(_) => println!("ERROR: Controller Window is busy"),
        }
            
    }

    pub fn set_data_observer(&mut self, obs: Observer<Message>) {
        match self.linker.clone().try_lock() {
            Ok(mut linker) => {
                linker.set_data_observer(obs);
            },
            Err(_) => println!("ERROR: ModuleLinker is still used by another process / thread"),
        }
    }
}