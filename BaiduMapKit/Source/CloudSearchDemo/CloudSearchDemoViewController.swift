//
//  CloudSearchDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/9.
//  Copyright © 2015年 baidu. All rights reserved.

/* 本示例代码使用了测试ak和测试数据，开发者在检索自己LBS数据之前，需替换 cloudLocalSearch.ak和cloudLocalSearch.geoTableId的值
    *
    * 1、替换cloudLocalSearch.ak的值：
    * （1）请访问http://lbsyun.baidu.com/apiconsole/key申请一个“服务端”的ak，其他类型的ak无效；
* （2）将申请的ak替换cloudLocalSearch.ak的值；
*
* 2、替换cloudLocalSearch.geoTableId值：
* （1）申请完服务端ak后访问http://lbsyun.baidu.com/datamanager/datamanage创建一张表；
* （2）在“表名称”处自由填写表的名称，如MyData，点击保存；
* （3）“创建”按钮右方将会出现形如“MyData(34195)”字样，其中的“34195”即为geoTableId的值；
* （4）添加或修改字段：点击“字段”标签修改和添加字段；
* （5）添加数据：
*  a、标注模式：“数据” ->“标注模式”，输入要添加的地址然后“百度一下”，点击地图蓝色图标，再点击保存即可；
*  b、批量模式： “数据” ->“批量模式”，可上传文件导入，具体文件格式要求请参见当页的“批量导入指南”；
* （6）选择左边“设置”标签，“是否发布到检索”选择“是”，然后"保存";
* （7）数据发布后，替换cloudLocalSearch.geoTableId的值即可；
* 备注：切记添加、删除或修改数据后要再次发布到检索，否则将会出现检索不到修改后数据的情况
*/

import UIKit

class CloudSearchDemoViewController: UIViewController, BMKMapViewDelegate, BMKCloudSearchDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    var cloudSearch: BMKCloudSearch!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        cloudSearch = BMKCloudSearch()
        
        // 初始化导航栏右侧按钮“说明”
        let customRightBarBuutonItem = UIBarButtonItem(title: "说明", style: .plain, target: self, action: #selector(CloudSearchDemoViewController.showGuide))
        self.navigationItem.rightBarButtonItem = customRightBarBuutonItem
        
        _mapView.zoomLevel = 14
        _mapView.isSelectedAnnotationViewFront = true

    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        cloudSearch.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
        cloudSearch.delegate = nil
    }

    // MARK: - IBAction
    
    //发起本地云检索
    @IBAction func onClickLocalSearch(_ sender: UIButton) {
        let cloudLocalSearch = BMKCloudLocalSearchInfo()
        cloudLocalSearch.ak = "B266f735e43ab207ec152deff44fec8b"
        cloudLocalSearch.geoTableId = 31869
        cloudLocalSearch.pageIndex = 0
        cloudLocalSearch.pageSize = 10
        cloudLocalSearch.region = "北京市"
        cloudLocalSearch.keyword = "天安门"
        let flag = cloudSearch.localSearch(with: cloudLocalSearch)
        if flag {
            print("本地云检索发送成功")
        }else {
            print("本地云检索发送失败")
        }

    }
    
    // 发起周边云检索
    @IBAction func onClickNearbySearch(_ sender: UIButton) {
        let cloudNearbySearch = BMKCloudNearbySearchInfo()
        cloudNearbySearch.ak = "B266f735e43ab207ec152deff44fec8b"
        cloudNearbySearch.geoTableId = 31869
        cloudNearbySearch.pageIndex = 0
        cloudNearbySearch.pageSize = 10
        cloudNearbySearch.location = "116.403402,39.915067"
        cloudNearbySearch.radius = 5
        cloudNearbySearch.keyword = "天安门"
        let flag = cloudSearch.nearbySearch(with: cloudNearbySearch)
        if flag {
            print("周边云检索发送成功")
        }else {
            print("周边云检索发送失败")
        }
    }
    
    // 发起矩形云检索
    @IBAction func onClickBoundSearch(_ sender: UIButton) {
        let cloudBoundSearch = BMKCloudBoundSearchInfo()
        cloudBoundSearch.ak = "B266f735e43ab207ec152deff44fec8b"
        cloudBoundSearch.geoTableId = 31869
        cloudBoundSearch.pageIndex = 0
        cloudBoundSearch.pageSize = 10
        cloudBoundSearch.bounds = "116.30,36.20;118.30,40.20"
        cloudBoundSearch.keyword = "天安门"
        let flag = cloudSearch.boundSearch(with: cloudBoundSearch)
        if flag {
            print("矩形云检索发送成功")
        }else {
            print("矩形云检索发送失败")
        }
    }
    
    // 发起详情云检索
    @IBAction func onClickDetailSearch(_ sender: UIButton) {
        let cloudDetailSearch = BMKCloudDetailSearchInfo()
        cloudDetailSearch.ak = "B266f735e43ab207ec152deff44fec8b"
        cloudDetailSearch.geoTableId = 31869
        cloudDetailSearch.uid = "19150264"
        let flag = cloudSearch.detailSearch(with: cloudDetailSearch)
        if flag {
            print("详情云检索发送成功")
        }else {
            print("详情云检索发送失败")
        }

    }
    
    // 说明按钮
    func showGuide() {
        let alertView = UIAlertController(title: "LBS.云检索－说明", message: "本示例使用了测试ak,开发者若需使用自有LBS数据,请留意代码中相关注释", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "确定", style: .default, handler: nil)
        alertView.addAction(okAction)
        self.present(alertView, animated: true, completion: nil)
    }
    
    // MARK: - BMKCloudSearchDelegate
    
    /**
    *返回云检索POI列表结果
    *@param poiResultList 云检索结果列表，成员类型为BMKCloudPOIList
    *@param type 返回结果类型： BMK_CLOUD_LOCAL_SEARCH,BMK_CLOUD_NEARBY_SEARCH,BMK_CLOUD_BOUND_SEARCH
    *@param error 错误号，@see BMKCloudErrorCode
    */
    func onGetCloudPoiResult(_ poiResultList: [Any]!, searchType type: Int32, errorCode error: Int32) {
        // 清除屏幕中所有的标注
        _mapView.removeAnnotations(_mapView.annotations)
        
        print("onGetCloudPoiResult error:\(error)");
        
        if error == BMK_CLOUD_NO_ERROR.rawValue {
            let result = poiResultList[0] as! BMKCloudPOIList
            for i in 0..<result.pois.count {
                let poi = result.pois[i] as! BMKCloudPOIInfo
                // 自定义字段
                if poi.customDict != nil && poi.customDict.count > 1 {
                    let customStringField = poi.customDict.object(forKey: "custom") as! String
                    print("custom field output = \(customStringField)")
                    let customDoubleField = poi.customDict.object(forKey: "double") as! Double
                    print("custom double field output = \(customDoubleField)")
                }
                
                let item = BMKPointAnnotation()
                let point = CLLocationCoordinate2DMake(Double(poi.longitude), Double(poi.latitude))
                item.coordinate = point
                item.title = poi.title
                _mapView.addAnnotation(item)
            }
            _mapView.showAnnotations(_mapView.annotations, animated: true)
        }
    }
    
    /**
     *返回云检索POI详情
     *@param poiDetailResult 类型为BMKCloudPOIInfo
     *@param type 返回结果类型： BMK_CLOUD_DETAIL_SEARCH
     *@param error 错误号，@see BMKCloudErrorCode
     */
    func onGetCloudPoiDetailResult(_ poiDetailResult: BMKCloudPOIInfo!, searchType type: Int32, errorCode error: Int32) {
        // 清除屏幕中所有的标注
        _mapView.removeAnnotations(_mapView.annotations)
        
        print("onGetCloudPoiResult error:\(error)");
        
        if error == BMK_CLOUD_NO_ERROR.rawValue {
            let item = BMKPointAnnotation()
            let point = CLLocationCoordinate2DMake(Double(poiDetailResult.longitude), Double(poiDetailResult.latitude))
            item.coordinate = point
            item.title = poiDetailResult.title
            _mapView.addAnnotation(item)

            _mapView.centerCoordinate = point
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
