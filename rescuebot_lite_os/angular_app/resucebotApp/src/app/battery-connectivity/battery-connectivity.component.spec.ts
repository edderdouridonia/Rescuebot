import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BatteryConnectivityComponent } from './battery-connectivity.component';

describe('BatteryConnectivityComponent', () => {
  let component: BatteryConnectivityComponent;
  let fixture: ComponentFixture<BatteryConnectivityComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ BatteryConnectivityComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(BatteryConnectivityComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
