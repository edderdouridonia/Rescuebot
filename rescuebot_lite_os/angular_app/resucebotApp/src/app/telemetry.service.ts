import { Injectable } from '@angular/core';
import { Client, Message } from '@stomp/stompjs';
import * as SockJS from 'sockjs-client';
import { BehaviorSubject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class TelemetryService {
  private client: Client;
  private sensors: { [key: string]: BehaviorSubject<any> } = {};

  constructor() {
    this.client = new Client({
      brokerURL: 'ws://localhost:15674/ws', // Change this to your RabbitMQ WebSocket URL
      connectHeaders: {
        login: 'guest', // Change this to your RabbitMQ username
        passcode: 'guest' // Change this to your RabbitMQ password
      },
      debug: (str) => {
        console.log(new Date(), str);
      },
      reconnectDelay: 5000,
      heartbeatIncoming: 4000,
      heartbeatOutgoing: 4000
    });

    this.client.activate();
    this.client.onConnect = () => {
      this.subscribeToAllSensors();
    };
  }

  subscribeToSensor(sensor: string): BehaviorSubject<any> {
    if (!this.sensors[sensor]) {
      this.sensors[sensor] = new BehaviorSubject<any>(null);
      this.client.onConnect = () => {
        this.client.subscribe(`/exchange/sensors/${sensor}`, (message: Message) => {
          this.sensors[sensor].next(JSON.parse(message.body));
        });
      };
    }
    return this.sensors[sensor];
  }

  // Subscribing to all sensors
  subscribeToAllSensors() {
    this.subscribeToSensor('environment');
    this.subscribeToSensor('luminosity');
    this.subscribeToSensor('uv');
    this.subscribeToSensor('airquality');
    this.subscribeToSensor('gyro');
    this.subscribeToSensor('air');
    // this.subscribeToSensor('video');
    // this.subscribeToSensor('proximity');
    // this.subscribeToSensor('ultrasonic');
  }
}
