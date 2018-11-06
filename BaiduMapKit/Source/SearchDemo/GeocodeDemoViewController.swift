//
//  GeocodeDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/6.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class GeocodeDemoViewController: UIViewController, BMKMapViewDelegate, BMKGeoCodeSearchDelegate, UITextFieldDelegate {
    
    @IBOutlet weak var _mapView: BMKMapView!
    @IBOutlet weak var cityField: UITextField!
    @IBOutlet weak var addressField: UITextField!
    @IBOutlet weak var latField: UITextField!
    @IBOutlet weak var lonField: UITextField!
    
    var geocodeSearch: BMKGeoCodeSearch!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        geocodeSearch = BMKGeoCodeSearch()
        
        // 界面初始化
        cityField.text = "北京"
        addressField.text = "海淀区上地十街10号"
        lonField.text = "116.403981"
        latField.text = "39.915101"
        cityField.delegate = self
        addressField.delegate = self
        lonField.delegate = self
        latField.delegate = self
        
        _mapView.zoomLevel = 14
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        geocodeSearch.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil // 不用时，置nil
        geocodeSearch.delegate = nil // 不用时，置nil
    }
    
    // MARK: - IBAction
    
    @IBAction func geoSearch(_ sender: UIButton) {
        let geocodeSearchOption = BMKGeoCodeSearchOption()
        geocodeSearchOption.city = cityField.text
        geocodeSearchOption.address = addressField.text
        let flag = geocodeSearch.geoCode(geocodeSearchOption)
        if flag {
            print("geo 检索发送成功")
        } else {
            print("geo 检索发送失败")
        }
    }
    
    @IBAction func reverseGeoSearch(_ sender: UIButton) {
        var lat = 0.0
        var lon = 0.0
        if (Double(latField.text!) != nil) {
            lat = Double(latField.text!)!
        }
        if (Double(lonField.text!) != nil) {
            lon = Double(lonField.text!)!
        }
        
        let reverseGeocodeSearchOption = BMKReverseGeoCodeSearchOption()
        reverseGeocodeSearchOption.location = CLLocationCoordinate2DMake(lat, lon)
        let flag = geocodeSearch.reverseGeoCode(reverseGeocodeSearchOption)
        if flag {
            print("反geo 检索发送成功")
        } else {
            print("反geo 检索发送失败")
        }
    }
    
    // MARK: - UITextFieldDelegate
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        cityField.resignFirstResponder()
        addressField.resignFirstResponder()
        lonField.resignFirstResponder()
        latField.resignFirstResponder()
        return true
    }
    
    // MARK: - BMKGeoCodeSearchDelegate
    
    /**
    *返回地址信息搜索结果
    *@param searcher 搜索对象
    *@param result 搜索结BMKGeoCodeSearch果
    *@param error 错误号，@see BMKSearchErrorCode
    */
    func onGetGeoCodeResult(_ searcher: BMKGeoCodeSearch!, result: BMKGeoCodeSearchResult!, errorCode error: BMKSearchErrorCode) {
        print("onGetGeoCodeResult error: \(error)")
        
        _mapView.removeAnnotations(_mapView.annotations)
        if error == BMK_SEARCH_NO_ERROR {
            let item = BMKPointAnnotation()
            item.coordinate = result.location
            item.title = addressField.text
            _mapView.addAnnotation(item)
            _mapView.centerCoordinate = result.location
            
            let showMessage = "纬度:\(item.coordinate.latitude)，经度:\(item.coordinate.longitude)"
            
            let alertView = UIAlertController(title: "正向地理编码", message: showMessage, preferredStyle: .alert)
            let okAction = UIAlertAction(title: "OK", style: .default, handler: nil)
            alertView.addAction(okAction)
            self.present(alertView, animated: true, completion: nil)
        }
    }
    
    /**
     *返回反地理编码搜索结果
     *@param searcher 搜索对象
     *@param result 搜索结果
     *@param error 错误号，@see BMKSearchErrorCode
     */
    func onGetReverseGeoCodeResult(_ searcher: BMKGeoCodeSearch!, result: BMKReverseGeoCodeSearchResult!, errorCode error: BMKSearchErrorCode) {
        print("onGetReverseGeoCodeResult error: \(error)")
        
        _mapView.removeAnnotations(_mapView.annotations)
        if error == BMK_SEARCH_NO_ERROR {
            let item = BMKPointAnnotation()
            item.coordinate = result.location
            item.title = result.address
            _mapView.addAnnotation(item)
            _mapView.centerCoordinate = result.location
            
            let alertView = UIAlertController(title: "反向地理编码", message: result.address, preferredStyle: .alert)
            let okAction = UIAlertAction(title: "OK", style: .default, handler: nil)
            alertView.addAction(okAction)
            self.present(alertView, animated: true, completion: nil)
        }
    }

    
    // MARK: - BMKMapViewDelegate
    
    /**
    *根据anntation生成对应的View
    *@param mapView 地图View
    *@param annotation 指定的标注
    *@return 生成的标注View
    */
    func mapView(_ mapView: BMKMapView!, viewFor annotation: BMKAnnotation!) -> BMKAnnotationView! {
        let AnnotationViewID = "renameMark"
        var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: AnnotationViewID) as! BMKPinAnnotationView?
        if annotationView == nil {
            annotationView = BMKPinAnnotationView(annotation: annotation, reuseIdentifier: AnnotationViewID)
            // 设置颜色
            annotationView!.pinColor = UInt(BMKPinAnnotationColorRed)
            // 从天上掉下的动画
            annotationView!.animatesDrop = true
            // 设置是否可以拖拽
            annotationView!.isDraggable = false
        }
        annotationView?.annotation = annotation
        return annotationView
    }
}
