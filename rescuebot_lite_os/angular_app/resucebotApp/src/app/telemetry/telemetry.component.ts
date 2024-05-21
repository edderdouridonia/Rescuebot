import { Component, OnInit, OnDestroy } from '@angular/core';
import { EChartsOption } from 'echarts';
import { TelemetryService } from '../telemetry.service';
import { BehaviorSubject, Subscription } from 'rxjs';

@Component({
  selector: 'app-telemetry',
  templateUrl: './telemetry.component.html',
  styleUrls: ['./telemetry.component.css']
})
export class TelemetryComponent implements OnInit, OnDestroy {

  environmentData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  luminosityData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  uvData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  airQualityData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  gyroData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  airData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  videoData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  proximityData: BehaviorSubject<any> = new BehaviorSubject<any>([]);
  ultrasonicData: BehaviorSubject<any> = new BehaviorSubject<any>([]);

  chartOptions: EChartsOption = {
    title: [
      { text: 'Temperature', top: '3%', left: 'center' },
      { text: 'Humidity', top: '28%', left: 'center' },
      { text: 'Luminosity', top: '53%', left: 'center' },
      { text: 'Gas Sensor', top: '78%', left: 'center' }
    ],
    grid: [
      { top: '7%', height: '18%' },
      { top: '32%', height: '18%' },
      { top: '57%', height: '18%' },
      { top: '82%', height: '18%' }
    ],
    xAxis: [
      { type: 'time', gridIndex: 0 },
      { type: 'time', gridIndex: 1 },
      { type: 'time', gridIndex: 2 },
      { type: 'time', gridIndex: 3 }
    ],
    yAxis: [
      { type: 'value', gridIndex: 0 },
      { type: 'value', gridIndex: 1 },
      { type: 'value', gridIndex: 2 },
      { type: 'value', gridIndex: 3 }
    ],
    series: [
      { name: 'Temperature', type: 'line', data: [], xAxisIndex: 0, yAxisIndex: 0 },
      { name: 'Humidity', type: 'line', data: [], xAxisIndex: 1, yAxisIndex: 1 },
      { name: 'Luminosity', type: 'line', data: [], xAxisIndex: 2, yAxisIndex: 2 },
      { name: 'Gas Sensor', type: 'line', data: [], xAxisIndex: 3, yAxisIndex: 3 }
    ]
  };

  private subscriptions: Subscription[] = [];

  constructor(private telemetryService: TelemetryService) {}

  ngOnInit(): void {
    this.environmentData = this.telemetryService.subscribeToSensor('environment');
    this.luminosityData = this.telemetryService.subscribeToSensor('luminosity');
    this.uvData = this.telemetryService.subscribeToSensor('uv');
    this.airQualityData = this.telemetryService.subscribeToSensor('airquality');
    this.gyroData = this.telemetryService.subscribeToSensor('gyro');
    this.airData = this.telemetryService.subscribeToSensor('air');
    // this.videoData = this.telemetryService.subscribeToSensor('video');
    // this.proximityData = this.telemetryService.subscribeToSensor('proximity');
    // this.ultrasonicData = this.telemetryService.subscribeToSensor('ultrasonic');

    this.subscriptions.push(
      this.environmentData.subscribe(data => {
        if (data) {
          this.updateChartData(0, data);
        }
      }),
      this.airData.subscribe(data => {
        if (data) {
          this.updateChartData(1, data);
        }
      }),
      this.luminosityData.subscribe(data => {
        if (data) {
          this.updateChartData(2, data);
        }
      }),
      this.airQualityData.subscribe(data => {
        if (data) {
          this.updateChartData(3, data);
        }
      })
    );
  }

  ngOnDestroy(): void {
    this.subscriptions.forEach(sub => sub.unsubscribe());
  }

  private updateChartData(index: number, data: any): void {
    const series = this.chartOptions.series as EChartsOption['series'];
    if (Array.isArray(series)) {
      const newData = series[index].data as any[];
      newData.push([new Date(), data.value]);
      this.chartOptions = {
        ...this.chartOptions,
        series: series.map((s, i) => i === index ? { ...s, data: newData } : s)
      };
    }
  }
}
