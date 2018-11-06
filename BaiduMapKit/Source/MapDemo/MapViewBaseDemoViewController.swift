//
//  MapViewBaseDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/2.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class MapViewBaseDemoViewController: UIViewController, BMKMapViewDelegate {
    var _mapView: BMKMapView?
    var enableCustomMap = false

    
    override func viewDidLoad() {
        super.viewDidLoad()
        let topHeight = (self.navigationController?.navigationBar.frame.height)! + UIApplication.shared.statusBarFrame.height

        _mapView = BMKMapView(frame: CGRect(x: 0, y: topHeight, width: self.view.frame.width, height: self.view.frame.height - topHeight))
        self.view.addSubview(_mapView!)
        
        addCustomGesture()//添加自定义手势
        
        //添加普通地图/个性化地图切换开关
        let segment = UISegmentedControl(items: ["normal", "custom"])
        segment.selectedSegmentIndex = 0
        segment.addTarget(self, action: #selector(MapViewBaseDemoViewController.changeMapAction(_:)), for: .valueChanged)
        self.navigationItem.rightBarButtonItem = UIBarButtonItem(customView: segment)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        BMKMapView.enableCustomMapStyle(enableCustomMap)
        _mapView?.viewWillAppear()
        _mapView?.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        BMKMapView.enableCustomMapStyle(false)//消失时，关闭个性化地图
        _mapView?.viewWillDisappear()
        _mapView?.delegate = nil
    }
    
    func changeMapAction(_ segment: UISegmentedControl) {
        /*
         *注：必须在BMKMapView对象初始化之前设置自定义地图样式，设置后会影响所有地图实例
         *设置方法：+ (void)customMapStyle:(NSString*) customMapStyleJsonFilePath;
         */
        enableCustomMap = segment.selectedSegmentIndex == 1
        //打开/关闭个性化地图
        BMKMapView.enableCustomMapStyle(enableCustomMap)
    }
    
    // MARK: - BMKMapViewDelegate
    
    /**
    *地图初始化完毕时会调用此接口
    *@param mapview 地图View
    */
    func mapViewDidFinishLoading(_ mapView: BMKMapView!) {
        let alertVC = UIAlertController(title: "", message: "BMKMapView控件初始化完成", preferredStyle: .alert)
        let alertAction = UIAlertAction(title: "知道了", style: .cancel, handler: nil)
        alertVC.addAction(alertAction)
        self .present(alertVC, animated: true, completion: nil)
    }
    
    // MARK: - 添加自定义手势 （若不自定义手势，不需要下面的代码）
    func addCustomGesture() {
        /*
        *注意：
        *添加自定义手势时，必须设置UIGestureRecognizer的cancelsTouchesInView 和 delaysTouchesEnded 属性设置为false，否则影响地图内部的手势处理
        */
        let tapGesturee = UITapGestureRecognizer(target: self, action: #selector(MapViewBaseDemoViewController.handleSingleTap(_:)))
        tapGesturee.cancelsTouchesInView = false
        tapGesturee.delaysTouchesEnded = false
        self.view.addGestureRecognizer(tapGesturee)
    }
    
    func handleSingleTap(_ tap: UITapGestureRecognizer) {
        NSLog("custom single tap handle")
    }
}
