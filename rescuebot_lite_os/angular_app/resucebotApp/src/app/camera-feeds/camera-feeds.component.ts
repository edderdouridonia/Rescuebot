import { Component } from '@angular/core';

@Component({
  selector: 'app-camera-feeds',
  templateUrl: './camera-feeds.component.html',
  styleUrls: ['./camera-feeds.component.css']
})
export class CameraFeedsComponent {
  recording: boolean = false;

  toggleRecording() {
    this.recording = !this.recording;
  }
}
