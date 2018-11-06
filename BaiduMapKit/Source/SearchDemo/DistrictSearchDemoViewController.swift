//
//  DistrictSearchDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 16/1/15.
//  Copyright © 2016年 baidu. All rights reserved.
//

import Foundation

class DistrictSearchDemoViewController: UIViewController, BMKMapViewDelegate, BMKDistrictSearchDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    @IBOutlet weak var cityField: UITextField!
    @IBOutlet weak var districtField: UITextField!
    
    var districtSearch: BMKDistrictSearch!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 初始化搜索服务
        districtSearch = BMKDistrictSearch()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        districtSearch.delegate = self
        requestDistrictSearch()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
        districtSearch.delegate = nil
    }
    
    @IBAction func districtSearch(_ sender: Any) {
        let option = BMKDistrictSearchOption()
        option.city = cityField.text
        option.district = districtField.text
        let flag = districtSearch.districtSearch(option)
        if flag {
            print("district检索发送成功")
        } else {
            print("district检索发送失败")
        }
    }
    //发起请求
    func requestDistrictSearch() {
        let option = BMKDistrictSearchOption()
        option.city = cityField.text
        option.district = districtField.text
        let flag = districtSearch.districtSearch(option)
        if flag {
            print("district检索发送成功")
        } else {
            print("district检索发送失败")
        }
    }

    // MARK: - BMKDistrictSearchDelegate
    /**
    *返回行政区域搜索结果
    *@param searcher 搜索对象
    *@param result 搜索结BMKDistrictSearch果
    *@param error 错误号，@see BMKSearchErrorCode
    */
    func onGetDistrictResult(_ searcher: BMKDistrictSearch!, result: BMKDistrictResult!, errorCode error: BMKSearchErrorCode) {
        print("onGetDistrictResult error: \(error)")
        _mapView.removeOverlays(_mapView.overlays)
        
        if error == BMK_SEARCH_NO_ERROR {
            print("\nname:\(result.name)\ncode:\(result.code)\ncenter latlon:\(result.center.latitude),\(result.center.longitude)");
            
            var flag = true
            for path in result.paths {
                let polygon = transferPathStringToPolygon(path as! String)
                if (polygon != nil) {
                    _mapView.add(polygon) // 添加overlay
                    if flag {
                        mapViewFitPolygon(polygon)
                        flag = false
                    }
                }
            }
        }
    }

    // MARK: - BMKMapViewDelegate
    /**
    *根据overlay生成对应的View
    *@param mapView 地图View
    *@param overlay 指定的overlay
    *@return 生成的覆盖物View
    */
    func mapView(_ mapView: BMKMapView!, viewFor overlay: BMKOverlay!) -> BMKOverlayView! {
        if (overlay as? BMKPolygon) != nil {
            let polygonView = BMKPolygonView(overlay: overlay)
            polygonView?.strokeColor = UIColor(red: 1, green: 0, blue: 0, alpha: 0.6)
            polygonView?.fillColor = UIColor(red: 1, green: 1, blue: 0, alpha: 0.4)
            polygonView?.lineWidth = 1
            polygonView?.lineDash = true
            return polygonView
        }
        return nil
    }
    
    
    // MARK: -
    //根据polygon设置地图范围
    func mapViewFitPolygon(_ polygon: BMKPolygon!) {
        if polygon.pointCount < 1 {
            return
        }
        
        let pt = polygon.points[0]
        var leftTopX = pt.x
        var leftTopY = pt.y
        var rightBottomX = pt.x
        var rightBottomY = pt.y
        
        for i in 1..<polygon.pointCount {
            let pt = polygon.points[Int(i)]
            leftTopX = pt.x < leftTopX ? pt.x : leftTopX;
            leftTopY = pt.y < leftTopY ? pt.y : leftTopY;
            rightBottomX = pt.x > rightBottomX ? pt.x : rightBottomX;
            rightBottomY = pt.y > rightBottomY ? pt.y : rightBottomY;
        }
        
        let rect = BMKMapRectMake(leftTopX, leftTopY, rightBottomX - leftTopX, rightBottomY - leftTopY)
        _mapView.visibleMapRect = rect
    }
    
    //根据path string 生成 BMKPolygon
    func transferPathStringToPolygon(_ path: String) -> BMKPolygon? {
        let pts = path.components(separatedBy: ";")
        if  pts.count < 1 {
            return nil
        }
        
        var points = Array(repeating: BMKMapPoint(x: 0, y: 0), count: pts.count)
        var index = 0
        for ptStr in pts {
            let range = ptStr.range(of: ",")
            let xStr = ptStr.substring(to: (range?.lowerBound)!)
            let yStr = ptStr.substring(from: (range?.upperBound)!)
            if xStr.characters.count > 0 && yStr.characters.count > 0  {
                points[index] = BMKMapPointMake(Double(xStr)!, Double(yStr)!)
                index += 1
            }
        }
        var polygon: BMKPolygon? = nil
        if index > 0 {
            polygon = BMKPolygon(points: &points, count: UInt(index))
        }
        return polygon
    }
    func mapView(_ mapView: BMKMapView!, onClickedMapBlank coordinate: CLLocationCoordinate2D) {
        cityField.resignFirstResponder()
        districtField.resignFirstResponder()
    }
    
}
