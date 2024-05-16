import { Component, OnInit } from '@angular/core';
import { EChartsOption } from 'echarts';

@Component({
  selector: 'app-telemetry',
  templateUrl: './telemetry.component.html',
  styleUrls: ['./telemetry.component.css']
})
export class TelemetryComponent implements OnInit {
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
      { type: 'category', boundaryGap: false, data: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'], gridIndex: 0 },
      { type: 'category', boundaryGap: false, data: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'], gridIndex: 1 },
      { type: 'category', boundaryGap: false, data: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'], gridIndex: 2 },
      { type: 'category', boundaryGap: false, data: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'], gridIndex: 3 }
    ],
    yAxis: [
      { type: 'value', gridIndex: 0 },
      { type: 'value', gridIndex: 1 },
      { type: 'value', gridIndex: 2 },
      { type: 'value', gridIndex: 3 }
    ],
    series: [
      {
        name: 'Temperature',
        type: 'line',
        data: [22, 24, 23, 25, 27, 26, 28],
        xAxisIndex: 0,
        yAxisIndex: 0
      },
      {
        name: 'Humidity',
        type: 'line',
        data: [55, 60, 58, 62, 65, 64, 63],
        xAxisIndex: 1,
        yAxisIndex: 1
      },
      {
        name: 'Luminosity',
        type: 'line',
        data: [300, 400, 350, 450, 500, 550, 600],
        xAxisIndex: 2,
        yAxisIndex: 2
      },
      {
        name: 'Gas Sensor',
        type: 'line',
        data: [0.5, 0.6, 0.55, 0.65, 0.7, 0.75, 0.8],
        xAxisIndex: 3,
        yAxisIndex: 3
      }
    ]
  };

  constructor() { }

  ngOnInit(): void {
  }
}
