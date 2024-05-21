import { Component } from '@angular/core';

@Component({
  selector: 'app-motion-control',
  templateUrl: './motion-control.component.html',
  styleUrls: ['./motion-control.component.css']
})
export class MotionControlComponent {
  motorStates: boolean[] = [false, false, false, false, false, false];
  directionStates = {
    up: false,
    down: false,
    left: false,
    right: false
  };
  powerLevel: number = 50;

  toggleMotor(motorIndex: number) {
    this.motorStates[motorIndex] = !this.motorStates[motorIndex];
  }

  toggleDirection(direction: string) {
    // this.directionStates[direction] = !this.directionStates[direction];
  }

  selectAll() {
    this.motorStates = this.motorStates.map(() => true);
  }

  deselectAll() {
    this.motorStates = this.motorStates.map(() => false);
  }
}
