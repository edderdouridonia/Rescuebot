import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CameraFeedsComponent } from './camera-feeds.component';

describe('CameraFeedsComponent', () => {
  let component: CameraFeedsComponent;
  let fixture: ComponentFixture<CameraFeedsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ CameraFeedsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CameraFeedsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
