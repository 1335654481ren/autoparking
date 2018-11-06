//
//  MapViewUISettingDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/5.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class MapViewUISettingDemoViewController: UIViewController, BMKMapViewDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        _mapView.showMapScaleBar = true
        _mapView.mapScaleBarPosition = CGPoint(x: 10, y: _mapView.frame.height - 45)
        setMapPadding()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
    }
    
    func setMapPadding() {
        ///地图预留边界，默认：UIEdgeInsetsZero。设置后，会根据mapPadding调整logo、比例尺、指南针的位置，以及targetScreenPt(BMKMapStatus.targetScreenPt)
        _mapView.mapPadding = UIEdgeInsetsMake(0, 0, 28, 0)
        
        let label = UILabel(frame: CGRect(x: 0, y: _mapView.frame.origin.y + _mapView.frame.size.height - 28, width: self.view.frame.size.width, height: 28))
        label.text = "已设置mapPadding为(0, 0, 28, 0)"
        label.font = UIFont.systemFont(ofSize: 13)
        label.textAlignment = .center
        label.backgroundColor = UIColor.white
        label.alpha = 0.7
        self.view.addSubview(label)
        self.view.bringSubview(toFront: label)
    }
    
    // MARK: - IBAction
    
    @IBAction func switchValueChanged(_ sender: UISwitch) {
        switch sender.tag {
        case 0://手势缩放
            _mapView.isZoomEnabled = sender.isOn
        case 1://平移
            _mapView.isScrollEnabled = sender.isOn
        case 2://比例尺
            _mapView.showMapScaleBar = sender.isOn
            _mapView.mapScaleBarPosition = CGPoint(x: 10, y: _mapView.frame.height - 70)
        case 3://所有手势
            _mapView.gesturesEnabled = sender.isOn
        case 4://双击手势
            _mapView.isChangeCenterWithDoubleTouchPointEnabled = sender.isOn
        default:
            return
        }

    }

    //改变指南针位置
    @IBAction func compassPositionChange(_ sender: UISegmentedControl) {
        var point: CGPoint
        if sender.selectedSegmentIndex == 0 {
            point = CGPoint(x: 10, y: 10)
        } else {
            point = CGPoint(x: _mapView.frame.width - 50, y: 10)
        }
        _mapView.compassPosition = point
    }
    
    // MARK: - BMKMapViewDelegate
    
}
