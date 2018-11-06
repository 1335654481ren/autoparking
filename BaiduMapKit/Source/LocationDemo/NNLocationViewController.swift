//
//  NNLocationViewController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/10/10.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import UIKit

class LocationViewController: UIViewController, BMKMapViewDelegate, BMKLocationManagerDelegate {
    
    var locationManager: BMKLocationManager!
    var userLocation: BMKUserLocation!
    
    weak var _mapView: BMKMapView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 添加按钮
        let customRightBarButtonItem = UIBarButtonItem(title: "自定义精度圈", style: .plain, target: self, action: #selector(LocationDemoViewController.customLocationAccuracyCircle))
        self.navigationItem.rightBarButtonItem = customRightBarButtonItem
        
        locationManager = BMKLocationManager()
        locationManager.delegate = self
        locationManager.coordinateType = BMKLocationCoordinateType.BMK09LL
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.activityType = CLActivityType.automotiveNavigation
        locationManager.locationTimeout = 10
        locationManager.allowsBackgroundLocationUpdates = true
        userLocation = BMKUserLocation()
        
        locationManager.startUpdatingHeading()
        locationManager.startUpdatingLocation()
        _mapView.showsUserLocation = false//先关闭显示的定位图层
        _mapView.userTrackingMode = BMKUserTrackingModeNone;//设置定位的状态
        _mapView.showsUserLocation = true//显示定位图层
        print("进入定位状态")
        _mapView.showsUserLocation = false
        _mapView.userTrackingMode = BMKUserTrackingModeFollow
        _mapView.showsUserLocation = true
        
        print("进入跟随态");
        _mapView.showsUserLocation = false
        _mapView.userTrackingMode = BMKUserTrackingModeFollow
        _mapView.showsUserLocation = true
        
        print("进入罗盘态");
        _mapView.showsUserLocation = false
        _mapView.userTrackingMode = BMKUserTrackingModeFollowWithHeading
        _mapView.showsUserLocation = true

    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.delegate = self
        _mapView.viewWillAppear()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        locationManager.delegate = nil
        _mapView.delegate = nil
        _mapView.viewWillDisappear()
    }
    
    //自定义精度圈
    func customLocationAccuracyCircle() {
        let param = BMKLocationViewDisplayParam()
        param.accuracyCircleStrokeColor = UIColor(red: 1, green: 0, blue: 0, alpha: 0.5)
        param.accuracyCircleFillColor = UIColor(red: 0, green: 1, blue: 0, alpha: 0.3)
        _mapView.updateLocationView(with: param)
    }
    
    // MARK: - BMKlocationManagerDelegate
    
    func bmkLocationManager(_ manager: BMKLocationManager, didUpdate heading: CLHeading?) {
        if let tempHeading = heading {
            userLocation.heading = tempHeading
            _mapView.updateLocationData(userLocation)
        }
    }
    
    func bmkLocationManager(_ manager: BMKLocationManager, didUpdate location: BMKLocation?, orError error: Error?) {
        if let temError = error {
            print("locError:{\(temError.localizedDescription)}")
        }
        
        if let temLocation = location {
            userLocation.location = temLocation.location;
            _mapView.updateLocationData(userLocation)
        }
    }
}

