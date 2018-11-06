//
//  ShortUrlShareDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/6.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit
import MessageUI

class ShortUrlShareDemoViewController: UIViewController, BMKMapViewDelegate, BMKShareURLSearchDelegate, BMKPoiSearchDelegate, BMKGeoCodeSearchDelegate, MFMessageComposeViewControllerDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    var shareURLSearch: BMKShareURLSearch!
    var geoCodeSearch: BMKGeoCodeSearch!
    var poiSearch: BMKPoiSearch!
    
    /// 名称
    var geoName: String!
    /// 地址
    var address: String!
    /// 短串
    var shortURL: String!
    /// 分享字符串
    var showMessage: String!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 添加说明按钮
        let customRightBarButtonItem = UIBarButtonItem(title: "说明", style: .plain, target: self, action: #selector(ShortUrlShareDemoViewController.showGuide))
        self.navigationItem.rightBarButtonItem = customRightBarButtonItem
        
        // 初始化搜索服务
        shareURLSearch = BMKShareURLSearch()
        geoCodeSearch = BMKGeoCodeSearch()
        poiSearch = BMKPoiSearch()

        _mapView.zoomLevel = 14
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        shareURLSearch.delegate = self
        geoCodeSearch.delegate = self
        poiSearch.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
        shareURLSearch.delegate = nil
        geoCodeSearch.delegate = nil
        poiSearch.delegate = nil
    }
    
    // MARK: - IBAction
    
    // 1. 点击[poi 搜索结果分享]，首先发起 poi 搜索请求
    @IBAction func poiShortUrlShare(_ sender: UIButton) {
        let citySearchOption = BMKPOICitySearchOption()
        citySearchOption.pageIndex = 0
        citySearchOption.pageSize = 10
        citySearchOption.city = "北京"
        citySearchOption.keyword = "故宫博物院"
        let flag = poiSearch.poiSearch(inCity: citySearchOption)
        if flag {
            print("城市内检索发送成功")
        }else {
            print("城市内检索发送失败")
        }
    }
    
    // 1.点击反向地理编码结果分享，首先发起反向地理编码搜索
    @IBAction func reverseGeoShortUrlShare(_ sender: UIButton) {
        let reverseGeocodeSearchOption = BMKReverseGeoCodeSearchOption()
        reverseGeocodeSearchOption.location = CLLocationCoordinate2DMake(39.915101, 116.403981)
        let flag = geoCodeSearch.reverseGeoCode(reverseGeocodeSearchOption)
        if flag {
            print("反向地理编码检索发送成功")
        }else {
            print("反向地理编码检索发送失败")
        }
    }
    
    ////路线规划短串分享url
    @IBAction func routePlanShortUrlShare(_ sender: UISegmentedControl) {
        let option = BMKRoutePlanShareURLOption()
        
        switch sender.selectedSegmentIndex {
        case 0://驾车
            option.routePlanType = BMK_ROUTE_PLAN_SHARE_URL_TYPE_DRIVE
            
        case 1://步行
            option.routePlanType = BMK_ROUTE_PLAN_SHARE_URL_TYPE_WALK
            
        case 2://骑行
            option.routePlanType = BMK_ROUTE_PLAN_SHARE_URL_TYPE_RIDE
            
        case 3://公交
            option.routePlanType = BMK_ROUTE_PLAN_SHARE_URL_TYPE_TRANSIT
            option.cityID = 131//当进行公交路线规划短串分享且起终点通过关键字指定时，必须指定
            option.routeIndex = 0//公交路线规划短串分享时使用，分享的是第几条线路
            
        default:
            break;
        }
        
        let fromNode = BMKPlanNode()
        fromNode.name = "百度大厦"
        fromNode.cityID = 131
        option.from = fromNode
        
        let toNode = BMKPlanNode()
        toNode.name = "天安门"
        toNode.cityID = 131
        option.to = toNode
        
        let flag = shareURLSearch.requestRoutePlanShareURL(option)
        if flag {
            print("routePlanShortUrlShare检索发送成功")
        } else {
            print("routePlanShortUrlShare检索发送失败")
        }
    }
    
    // 显示说明
    func showGuide() {
        let alertView = UIAlertController(title: "短串分享－说明", message: "短串分享是将POI点、反Geo点、路线规划，生成短链接串，此链接可通过短信等形式分享给好友，好友在终端设备点击此链接可快速打开Web地图、百度地图客户端进行信息展示", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "确定", style: .default, handler: nil)
        alertView.addAction(okAction)
        self.present(alertView, animated: true, completion: nil)
    }
    
    // MARK: - BMKPoiSearchDelegate
    /**
    *返回POI搜索结果
    *@param searcher 搜索对象
    *@param poiResult 搜索结果列表
    *@param errorCode 错误号，@see BMKSearchErrorCode
    */
    func onGetPoiResult(_ searcher: BMKPoiSearch!, result poiResult: BMKPOISearchResult!, errorCode: BMKSearchErrorCode) {
        // 清除屏幕中所有的标注
        _mapView.removeAnnotations(_mapView.annotations)
        
        if errorCode == BMK_SEARCH_NO_ERROR {
            if poiResult.poiInfoList.count > 0 {
                // 获取第一个 poi 点的数据
                let poi = poiResult.poiInfoList[0] as! BMKPoiInfo
                let item = BMKPointAnnotation()
                item.coordinate = poi.pt
                item.title = poi.name
                _mapView.addAnnotation(item)
                _mapView.centerCoordinate = poi.pt
                
                // 保存数据以便用于分享
                geoName = poi.name
                address = poi.address
                // 发起短串搜索获取 poi 分享 URL
                let detailShareURLSearchOption = BMKPoiDetailShareURLOption()
                detailShareURLSearchOption.uid = poi.uid
                let flag = shareURLSearch.requestPoiDetailShareURL(detailShareURLSearchOption)
                if flag {
                    print("详情 URL 检索发送成功！")
                }else {
                    print("详情 URL 检索发送失败！")
                }
            }
        }
    }
    
    // MARK: - BMKGeoCodeSearchDelegate
    /**
    *返回反地理编码搜索结果
    *@param searcher 搜索对象
    *@param result 搜索结果
    *@param error 错误号，@see BMKSearchErrorCode
    */
    func onGetReverseGeoCodeResult(_ searcher: BMKGeoCodeSearch!, result: BMKReverseGeoCodeSearchResult!, errorCode error: BMKSearchErrorCode) {
        // 清除屏幕中所有的标注
        _mapView.removeAnnotations(_mapView.annotations)
        
        if error == BMK_SEARCH_NO_ERROR {
            let item = BMKPointAnnotation()
            item.coordinate = result.location
            item.title = result.address
            _mapView.addAnnotation(item)
            _mapView.centerCoordinate = result.location
            
            // 保存数据以便用于分享
            // 名字——气泡上所显示的名字，可以自定义
            geoName = "自定义的气泡名称"
            // 地址
            address = result.address
            // 发起短串搜索功能获取反向地理位置分享 URL
            let option = BMKLocationShareURLOption()
            option.snippet = address
            option.name = geoName
            option.location = result.location
            let flag = shareURLSearch.requestLocationShareURL(option)
            if flag {
                print("反向地理位置 URL 检索发送成")
            }else {
                print("反向地理位置 URL 检索发送失败")
            }
        }
    }

    // MARK: - BMKShareURLSearchDelegate
    /**
    *获取poi详情短串分享url
    *异步函数，返回结果在BMKShareUrlSearchDelegate的onGetPoiDetailShareURLResult通知
    *@param poiDetailShareUrlSearchOption poi详情短串分享检索信息类
    *@return 成功返回YES，否则返回NO
    */
    func onGetPoiDetailShareURLResult(_ searcher: BMKShareURLSearch!, result: BMKShareURLResult!, errorCode error: BMKSearchErrorCode) {
        if error == BMK_SEARCH_NO_ERROR {
            shortURL = result.url
            showMessage = "这里是\(geoName)\r\n\(address)\r\n\(shortURL)"
            let alertView = UIAlertController(title: "短串分享", message:  showMessage, preferredStyle: .alert)
            let shareAction = UIAlertAction(title: "分享", style: .default, handler: { Void in
                if MFMessageComposeViewController.canSendText() {
                    let message = MFMessageComposeViewController()
                    message.messageComposeDelegate = self
                    message.body = self.showMessage
                    self.present(message, animated: true, completion: nil)
                }else {
                    let alertView = UIAlertController(title: "当前设备暂时没有办法发送短信", message: nil, preferredStyle: .alert)
                    let okAction = UIAlertAction(title: "确定", style: .default, handler: nil)
                    alertView.addAction(okAction)
                    self.present(alertView, animated: true, completion: nil)
                }
            })
            let cancelAction = UIAlertAction(title: "取消", style: .cancel, handler: nil)
            alertView.addAction(shareAction)
            alertView.addAction(cancelAction)
            self.present(alertView, animated: true, completion: nil)
        }
    }
    
    /**
     *获取反geo短串分享url
     *异步函数，返回结果在BMKShareUrlSearchDelegate的onGetLocationShareURLResult通知
     *@param reverseGeoShareUrlSearchOption 反geo短串分享检索信息类
     *@return 成功返回YES，否则返回NO
     */
    func onGetLocationShareURLResult(_ searcher: BMKShareURLSearch!, result: BMKShareURLResult!, errorCode error: BMKSearchErrorCode) {
        if error == BMK_SEARCH_NO_ERROR {
            shortURL = result.url
            showMessage = "这里是\(geoName)\r\n\(address)\r\n\(shortURL)"
            let alertView = UIAlertController(title: "短串分享", message:  showMessage, preferredStyle: .alert)
            let shareAction = UIAlertAction(title: "分享", style: .default, handler: { Void in
                if MFMessageComposeViewController.canSendText() {
                    let message = MFMessageComposeViewController()
                    message.messageComposeDelegate = self
                    message.body = self.showMessage
                    self.present(message, animated: true, completion: nil)
                }else {
                    let alertView = UIAlertController(title: "当前设备暂时没有办法发送短信", message: nil, preferredStyle: .alert)
                    let okAction = UIAlertAction(title: "确定", style: .default, handler: nil)
                    alertView.addAction(okAction)
                    self.present(alertView, animated: true, completion: nil)
                }
            })
            let cancelAction = UIAlertAction(title: "取消", style: .cancel, handler: nil)
            alertView.addAction(shareAction)
            alertView.addAction(cancelAction)
            self.present(alertView, animated: true, completion: nil)
        }
    }
    
    /**
     *返回路线规划分享url
     *@param searcher 搜索对象
     *@param result 返回结果
     *@param error 错误号，@see BMKSearchErrorCode
     */
    func onGetRoutePlanShareURLResult(_ searcher: BMKShareURLSearch!, result: BMKShareURLResult!, errorCode error: BMKSearchErrorCode) {
        if error == BMK_SEARCH_NO_ERROR {
            shortURL = result.url
            showMessage = "分享的路线规划短串为：\n\(shortURL)"
            let alertView = UIAlertController(title: "短串分享", message:  showMessage, preferredStyle: .alert)
            let shareAction = UIAlertAction(title: "分享", style: .default, handler: { Void in
                if MFMessageComposeViewController.canSendText() {
                    let message = MFMessageComposeViewController()
                    message.messageComposeDelegate = self
                    message.body = self.showMessage
                    self.present(message, animated: true, completion: nil)
                } else {
                    let alertView = UIAlertController(title: "当前设备暂时没有办法发送短信", message: nil, preferredStyle: .alert)
                    let okAction = UIAlertAction(title: "确定", style: .default, handler: nil)
                    alertView.addAction(okAction)
                    self.present(alertView, animated: true, completion: nil)
                }
            })
            let cancelAction = UIAlertAction(title: "取消", style: .cancel, handler: nil)
            alertView.addAction(shareAction)
            alertView.addAction(cancelAction)
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

    
    // MARK: - Message 相关协议实现
    func messageComposeViewController(_ controller: MFMessageComposeViewController, didFinishWith result: MessageComposeResult) {
        switch result.rawValue {
        case 0:
            // 用户自己取消，不用提醒
            print("用户取消发送")
        case 1:
            print("用户提交发送")
        case 2:
            print("发送失败")
        default:
            print("短信没有发送")
        }
        self.dismiss(animated: true, completion: nil)
    }

}
