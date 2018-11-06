//
//  BusLineSearchViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/6.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class BusLineSearchViewController: UIViewController, BMKMapViewDelegate, BMKBusLineSearchDelegate, BMKPoiSearchDelegate, UITextFieldDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    @IBOutlet weak var cityField: UITextField!
    @IBOutlet weak var nameField: UITextField!
    
    var buslineSearch: BMKBusLineSearch!
    var poiSearch: BMKPoiSearch!
    
    var poiUidArray: [String]!
    var currentIndex: Int = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        buslineSearch = BMKBusLineSearch()
        poiSearch = BMKPoiSearch()
        
        poiUidArray = [String]()
        
        cityField.text = "北京"
        nameField.text = "320"
        cityField.delegate = self
        nameField.delegate = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        buslineSearch.delegate = self
        poiSearch.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
        buslineSearch.delegate = nil
        poiSearch.delegate = nil
    }
    
    // MARK: - IBAction
    
    @IBAction func buslineSearch(_ sender: AnyObject) {
        poiUidArray.removeAll(keepingCapacity: false)
        citySearch()
    }
    
    @IBAction func buslineSearchNext(_ sender: AnyObject) {
        if poiUidArray.count > 0 {
            currentIndex += 1
            if currentIndex >= poiUidArray.count {
                currentIndex -= poiUidArray.count
            }
            bslSearch()
        }else {
            citySearch()
        }
    }
    
    func citySearch() {
        let citySearchOption = BMKPOICitySearchOption()
        citySearchOption.pageIndex = 0
        citySearchOption.pageSize = 10
        citySearchOption.city = cityField.text
        citySearchOption.keyword = nameField.text
        
        let flag = poiSearch.poiSearch(inCity: citySearchOption)
        if flag {
            print("城市内检索发送成功")
        }else {
            print("城市内检索发送失败")
        }
    }
    
    func bslSearch() {
        let busLineUid = poiUidArray[currentIndex] as String
        let buslineSearchOption = BMKBusLineSearchOption()
        buslineSearchOption.city = cityField.text
        buslineSearchOption.busLineUid = busLineUid
        
        let flag = buslineSearch.busLineSearch(buslineSearchOption)
        if flag {
            print("公交线路检索发送成功")
        }else {
            print("公交线路检索发送失败")
        }

    }
    
    // MARK: - UITextFieldDelegate
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        cityField.resignFirstResponder()
        nameField.resignFirstResponder()
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
        
        if errorCode == BMK_SEARCH_NO_ERROR {
            var findBusline = false
            for i in 0..<poiResult.poiInfoList.count {
                let poi = poiResult.poiInfoList[i] as? BMKPoiInfo
                if let poiUid = poi?.uid {
                    findBusline = true
                    poiUidArray.append(poiUid)
                }
            }
            // 开始 busline 详情搜索
            if findBusline {
                currentIndex = 0
                bslSearch()
            }
        }
    }

    // MARK: - BMKBusLineSearchDelegate
    
    /**
    *返回busdetail搜索结果
    *@param searcher 搜索对象
    *@param busLineResult 搜索结果
    *@param error 错误号，@see BMKSearchErrorCode
    */
    func onGetBusDetailResult(_ searcher: BMKBusLineSearch!, result busLineResult: BMKBusLineResult!, errorCode error: BMKSearchErrorCode) {
        print("onGetBusDetailResult: \(error)")
        
        _mapView.removeAnnotations(_mapView.annotations)
        _mapView.removeOverlays(_mapView.overlays)
        
        if error == BMK_SEARCH_NO_ERROR {
            var item = RouteAnnotation()
            
            // 站点信息
            let size = busLineResult.busStations.count
            for i in 0..<size {
                let station = busLineResult.busStations[i] as! BMKBusStation
                item = RouteAnnotation()
                item.coordinate = station.location
                item.title = station.title
                item.type = 2
                _mapView.addAnnotation(item)
            }
            
            // 路段信息
            var index = 0
            for i in 0..<busLineResult.busSteps.count {
                let step = busLineResult.busSteps[i] as! BMKBusStep
                index += Int(step.pointsCount)
            }
            
            // 直角坐标划线
            var tempPoints = Array(repeating: BMKMapPoint(x: 0, y: 0), count: index)
            var k = 0
            for i in 0..<busLineResult.busSteps.count {
                let step = busLineResult.busSteps[i] as! BMKBusStep
                for j in 0..<step.pointsCount {
                    let point = BMKMapPoint(x: step.points[Int(j)].x, y: step.points[Int(j)].y)
                    tempPoints[k] = point
                    k += 1
                }
            }
            
            let polyLine = BMKPolyline(points: &tempPoints, count: UInt(index))
            _mapView.add(polyLine)
            
            mapViewFitPolyLine(polyLine)
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
        if let routeAnnotation = annotation as! RouteAnnotation? {
            return getViewForRouteAnnotation(routeAnnotation)
        }
        return nil
    }
    
    /**
     *根据overlay生成对应的View
     *@param mapView 地图View
     *@param overlay 指定的overlay
     *@return 生成的覆盖物View
     */
    func mapView(_ mapView: BMKMapView!, viewFor overlay: BMKOverlay!) -> BMKOverlayView! {
        if overlay as! BMKPolyline? != nil {
            let polylineView = BMKPolylineView(overlay: overlay as! BMKPolyline)
            polylineView?.strokeColor = UIColor(red: 0, green: 0, blue: 1, alpha: 0.7)
            polylineView?.lineWidth = 3
            return polylineView
        }
        return nil
    }
    
    // MARK: -
    
    func getViewForRouteAnnotation(_ routeAnnotation: RouteAnnotation!) -> BMKAnnotationView? {
        var view: BMKAnnotationView?
        
        var imageName: String?
        switch routeAnnotation.type {
        case 0:
            imageName = "nav_start"
        case 1:
            imageName = "nav_end"
        case 2:
            imageName = "nav_bus"
        case 3:
            imageName = "nav_rail"
        case 4:
            imageName = "direction"
        case 5:
            imageName = "nav_waypoint"
        default:
            return nil
        }
        let identifier = "\(String(describing: imageName))_annotation"
        
        view = _mapView.dequeueReusableAnnotationView(withIdentifier: identifier)
        if view == nil {
            view = BMKAnnotationView(annotation: routeAnnotation, reuseIdentifier: identifier)
            view?.centerOffset = CGPoint(x: 0, y: -(view!.frame.size.height * 0.5))
            view?.canShowCallout = true
        }
        
        view?.annotation = routeAnnotation
        
        let bundlePath = (Bundle.main.resourcePath)! + "/mapapi.bundle/"
        let bundle = Bundle(path: bundlePath)
        var tmpBundle : String?
        tmpBundle = (bundle?.resourcePath)! + "/images/icon_\(imageName!).png"
        
        if let imagePath = tmpBundle {
            var image = UIImage(contentsOfFile: imagePath)
            if routeAnnotation.type == 4 {
                image = imageRotated(image, degrees: routeAnnotation.degree)
            }
            if image != nil {
                view?.image = image
            }
        }
        
        return view
    }
    
    
    //根据polyline设置地图范围
    func mapViewFitPolyLine(_ polyline: BMKPolyline!) {
        if polyline.pointCount < 1 {
            return
        }
        
        let pt = polyline.points[0]
        var leftTopX = pt.x
        var leftTopY = pt.y
        var rightBottomX = pt.x
        var rightBottomY = pt.y
        
        for i in 1..<polyline.pointCount {
            let pt = polyline.points[Int(i)]
            leftTopX = pt.x < leftTopX ? pt.x : leftTopX;
            leftTopY = pt.y < leftTopY ? pt.y : leftTopY;
            rightBottomX = pt.x > rightBottomX ? pt.x : rightBottomX;
            rightBottomY = pt.y > rightBottomY ? pt.y : rightBottomY;
        }
        
        let rect = BMKMapRectMake(leftTopX, leftTopY, rightBottomX - leftTopX, rightBottomY - leftTopY)
        _mapView.visibleMapRect = rect
    }
    
    //旋转图片
    func imageRotated(_ image: UIImage!, degrees: Int!) -> UIImage {
        let width = image.cgImage?.width
        let height = image.cgImage?.height
        let rotatedSize = CGSize(width: width!, height: height!)
        UIGraphicsBeginImageContext(rotatedSize);
        let bitmap = UIGraphicsGetCurrentContext();
        bitmap?.translateBy(x: rotatedSize.width/2, y: rotatedSize.height/2);
        bitmap?.rotate(by: CGFloat(Double(degrees) * Double.pi / 180.0));
        bitmap?.rotate(by: CGFloat(Double.pi));
        bitmap?.scaleBy(x: -1.0, y: 1.0);
        bitmap?.draw(image.cgImage!, in: CGRect(x: -rotatedSize.width/2, y: -rotatedSize.height/2, width: rotatedSize.width, height: rotatedSize.height));
        let newImage = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();
        return newImage!;
    }

}
