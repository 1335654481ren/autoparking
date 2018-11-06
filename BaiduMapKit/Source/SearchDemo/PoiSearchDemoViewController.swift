//
//  PoiSearchDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/6.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class PoiSearchDemoViewController: UIViewController, BMKMapViewDelegate, BMKPoiSearchDelegate, UITextFieldDelegate {
    
    @IBOutlet weak var _mapView: BMKMapView!
    @IBOutlet weak var cityField: UITextField!
    @IBOutlet weak var keywordField: UITextField!
    
    var poiSearch: BMKPoiSearch!
    var currPageIndex: Int32 = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        poiSearch = BMKPoiSearch()
        
        cityField.text = "北京"
        keywordField.text = "餐厅"
        cityField.delegate = self
        keywordField.delegate = self
        
        _mapView.zoomLevel = 13
        _mapView.isSelectedAnnotationViewFront = true
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        poiSearch.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
        poiSearch.delegate = nil
    }
    
    // MARK: - IBAction
    
    @IBAction func searchAction(_ sender: UIButton) {
        currPageIndex = 0
        sendPoiSearchRequest()
    }
    
    @IBAction func nextAction(_ sender: UIButton) {
        currPageIndex += 1
        sendPoiSearchRequest()
    }
    
    func sendPoiSearchRequest() {
        let citySearchOption = BMKPOICitySearchOption()
        citySearchOption.pageIndex = Int(currPageIndex)
        citySearchOption.pageSize = 10
        citySearchOption.city = cityField.text
        citySearchOption.keyword = keywordField.text
        if poiSearch.poiSearch(inCity: citySearchOption) {
            print("城市内检索发送成功！")
        }else {
            print("城市内检索发送失败！")
        }
    }
    
    // MARK: - UITextFieldDelegate
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        cityField.resignFirstResponder()
        keywordField.resignFirstResponder()
        return true
    }
    
    // MARK: - BMKPoiSearchDelegate
    /**
    *返回POI搜索结果
    *@param searcher 搜索对象
    *@param poiResult 搜索结果列表
    *@param errorCode 错误号，@see BMKSearchErrorCode
    */
    func onGetPoiResult(_ searcher: BMKPoiSearch!, result poiResult: BMKPOISearchResult!, errorCode: BMKSearchErrorCode) {
        print("onGetPoiResult code: \(errorCode)");
        
        // 清除屏幕中所有的 annotation
        _mapView.removeAnnotations(_mapView.annotations)
        
        if errorCode == BMK_SEARCH_NO_ERROR {
            var annotations = [BMKPointAnnotation]()
            for i in 0..<poiResult.poiInfoList.count {
                let poi = poiResult.poiInfoList[i] as! BMKPoiInfo
                let item = BMKPointAnnotation()
                item.coordinate = poi.pt
                item.title = poi.name
                annotations.append(item)
            }
            _mapView.addAnnotations(annotations)
            _mapView.showAnnotations(annotations, animated: true)
        } else if errorCode == BMK_SEARCH_AMBIGUOUS_KEYWORD {
            print("检索词有歧义")
        } else {
            // 各种情况的判断……
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
