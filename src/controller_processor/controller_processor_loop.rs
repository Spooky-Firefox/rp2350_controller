use crate::controller_processor::controller::{Controller, PIDController};

use super::{kalman_filter};


use fugit::TimerInstantU64;
use rp235x_pac::Peripherals;
pub const STACK_SIZE: usize = 65536;

pub fn init(peripherals: &mut Peripherals, sio: &mut rp235x_hal::sio::Sio, stack : rp235x_hal::multicore::Stack<65536>) {

    let mut mc = rp235x_hal::multicore::Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    mono
    let _ = core1.spawn(stack.take().unwrap(), move || core1_task(time));
    // Initialize the second processor core and start the main loop
    // This is platform specific, so it is not implemented here
}


///! This module contains the main loop for the controller processor. It is responsible for running the kalman filter and controller on the second processor core
pub fn core1_task(t0: TimerInstantU64) {
    // Initialize the communication with the main processor core, and any other necessary peripherals


    // Init the kalman filter and controller
    
    let kalman_const = kalman_filter::EkfConst { 
        l: 0.2, 
        speed_ratio: 1.0, 
        q_pos: 0.01, 
        q_theta: 0.01, 
        q_v: 0.01, 
        r_omega: 0.01, 
        eps_v: 0.01, 
        dt_max_us: 20000,
    };

    let x0: [f32; 4] = [0.0; 4];

    let p0: [f32; 16] = [   1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                0.0, 0.01, 1.0, 0.0,
                                0.0, 0.0, 0.0, 1.0];

    let mut filter: kalman_filter::EkfFilter = kalman_filter::EkfFilter::new(kalman_const, x0, p0,t0);

    let mut controller = PIDController {    
        kp: 1.0,
        ki: 0.0,
        kd: 0.0,
        setpoint: [0.0; 4],
        integral: [0.0; 4],
        previous_error: [0.0; 4],
    };

    main_loop(&mut controller, &mut filter);
}


fn main_loop(controller: &mut PIDController, filter: &mut kalman_filter::EkfFilter) {
     // This loop waits for new sensor data from the main processor core, runs the kalman filter and controller, and sends the control signals back to the main processor core
    loop {
        
    }
}
