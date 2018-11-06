//
//  MultiMapViewDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/3.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class MultiMapViewDemoViewController: UIViewController, BMKMapViewDelegate {
    var _mapView1: BMKMapView?
    var _mapView2: BMKMapView?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        let topHeight = (self.navigationController?.navigationBar.frame.height)! + UIApplication.shared.statusBarFrame.height
        let height = (self.view.frame.height - topHeight - 30) / 2
        let width = self.view.frame.width - 20
        
        _mapView1 = BMKMapView(frame: CGRect(x: 10, y: topHeight + 10, width: width, height: height))
        _mapView1?.mapType = BMKMapType.satellite
        _mapView1?.logoPosition = BMKLogoPositionLeftBottom
        self.view.addSubview(_mapView1!)
        
        _mapView2 = BMKMapView(frame: CGRect(x: 10, y: topHeight + height + 20, width: width, height: height))
        _mapView2?.logoPosition = BMKLogoPositionRightBottom
        self.view.addSubview(_mapView2!)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView1?.viewWillAppear()
        _mapView1?.delegate = self
        _mapView2?.viewWillAppear()
        _mapView2?.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView1?.viewWillDisappear()
        _mapView1?.delegate = nil
        _mapView2?.viewWillDisappear()
        _mapView2?.delegate = nil
    }
    
    // MARK: - BMKMapViewDelegate
    
    /**
    *点中底图空白处会回调此接口
    *@param mapview 地图View
    *@param coordinate 空白处坐标点的经纬度
    */
    func mapView(_ mapView: BMKMapView!, onClickedMapBlank coordinate: CLLocationCoordinate2D) {
        if mapView == _mapView1 {
            NSLog("mapView1-onClickedMapBlank");
        } else if mapView == _mapView2 {
            NSLog("mapView2-onClickedMapBlank");
        }
    }
    
    /**
     *双击地图时会回调此接口
     *@param mapview 地图View
     *@param coordinate 返回双击处坐标点的经纬度
     */
    func mapview(_ mapView: BMKMapView!, onDoubleClick coordinate: CLLocationCoordinate2D) {
        if mapView == _mapView1 {
            NSLog("mapView1-onDoubleClick");
        } else if mapView == _mapView2 {
            NSLog("mapView2-onDoubleClick");
        }
    }

}
