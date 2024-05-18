import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MotionControlComponent } from './motion-control.component';

describe('MotionControlComponent', () => {
  let component: MotionControlComponent;
  let fixture: ComponentFixture<MotionControlComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MotionControlComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MotionControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
