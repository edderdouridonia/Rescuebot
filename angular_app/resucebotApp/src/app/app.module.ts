import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { FormsModule } from '@angular/forms';  // Import FormsModule
import { MotionControlComponent } from './motion-control/motion-control.component';
import { CameraFeedsComponent } from './camera-feeds/camera-feeds.component';
import { CommsComponent } from './comms/comms.component';
import { BatteryConnectivityComponent } from './battery-connectivity/battery-connectivity.component';
import { TelemetryComponent } from './telemetry/telemetry.component';
import { ModeSelectionComponent } from './mode-selection/mode-selection.component';
import { NgxEchartsModule } from 'ngx-echarts';

@NgModule({
  declarations: [
    AppComponent,
    MotionControlComponent,
    CameraFeedsComponent,
    CommsComponent,
    BatteryConnectivityComponent,
    TelemetryComponent,
    ModeSelectionComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    FormsModule,  // Add FormsModule
    NgxEchartsModule.forRoot({
      echarts: () => import('echarts')
    })
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
