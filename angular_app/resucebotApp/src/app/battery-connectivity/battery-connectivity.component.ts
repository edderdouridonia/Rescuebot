import { Component } from '@angular/core';

@Component({
  selector: 'app-battery-connectivity',
  templateUrl: './battery-connectivity.component.html',
  styleUrls: ['./battery-connectivity.component.css']
})
export class BatteryConnectivityComponent {
  batteryLevel: number = 50;
  wirelessCharging: boolean = false;
  transmitting: boolean = false;
  receiving: boolean = false;

  toggleWirelessCharging() {
    this.wirelessCharging = !this.wirelessCharging;
  }

  toggleTransmitting() {
    this.transmitting = !this.transmitting;
  }

  toggleReceiving() {
    this.receiving = !this.receiving;
  }

  getBatteryBars() {
    const bars = [];
    const totalBars = 10;
    const filledBars = Math.floor((this.batteryLevel / 100) * totalBars);

    for (let i = 0; i < totalBars; i++) {
      bars.push(i < filledBars);
    }
    return bars;
  }
}
