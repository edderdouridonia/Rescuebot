import { Component } from '@angular/core';

@Component({
  selector: 'app-motion-control',
  templateUrl: './motion-control.component.html',
  styleUrls: ['./motion-control.component.css']
})
export class MotionControlComponent {
  motorStates: boolean[] = [false, false, false, false, false, false];
  powerLevel: number = 50;
  speedLevel: number = 50; // New state for second speed control
  joystickDirection: string = ''; // New state for joystick direction

  toggleMotor(motorIndex: number) {
    this.motorStates[motorIndex] = !this.motorStates[motorIndex];
  }

  selectAll() {
    this.motorStates = this.motorStates.map(() => true);
  }

  deselectAll() {
    this.motorStates = this.motorStates.map(() => false);
  }

  setJoystickDirection(direction: string) {
    this.joystickDirection = direction;
  }
}
