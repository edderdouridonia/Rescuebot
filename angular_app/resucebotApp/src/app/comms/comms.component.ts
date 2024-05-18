import { Component } from '@angular/core';

@Component({
  selector: 'app-comms',
  templateUrl: './comms.component.html',
  styleUrls: ['./comms.component.css']
})
export class CommsComponent {
  micOn: boolean = false;
  speakerOn: boolean = false;

  toggleMic() {
    this.micOn = !this.micOn;
  }

  toggleSpeaker() {
    this.speakerOn = !this.speakerOn;
  }
}
