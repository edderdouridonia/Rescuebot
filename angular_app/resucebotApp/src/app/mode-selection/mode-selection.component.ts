import { Component } from '@angular/core';

@Component({
  selector: 'app-mode-selection',
  templateUrl: './mode-selection.component.html',
  styleUrls: ['./mode-selection.component.css']
})
export class ModeSelectionComponent {
  mode: 'autonomous' | 'manual' = 'manual';

  toggleMode(selectedMode: 'autonomous' | 'manual') {
    this.mode = selectedMode;
  }
}
