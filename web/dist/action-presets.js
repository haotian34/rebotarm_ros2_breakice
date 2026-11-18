export const actionBusy=state=>['running','stopping'].includes(state?.action_motion?.status);
// A multiplier scales time only. Preview frames come from the shared ROS state machines.
export const actionSpeedLabel=speed=>`${Number(speed).toFixed(2)}×`;
