//
//  NNTestViewController.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.
//

import UIKit
import Alamofire
import SwiftyJSON
import WebSocket

class NNBaiduMapViewController: UIViewController ,BMKMapViewDelegate ,BMKLocationManagerDelegate{
    var socket: WebSocket!
    var _mapView: BMKMapView?
    var locationManager: BMKLocationManager!
    var userLocation: BMKUserLocation!
    var lockedScreenAnnotation: BMKPointAnnotation?
    var once_flag = 1
    var baidu_index = 0
    //登录框状态
    var showType:AnnotationShowType = AnnotationShowType.NONE
    var carAnnotation: BMKPointAnnotation?
    var carNameArry = [String]()
    var carArry = [String : BMKPointAnnotation ]()
    
    var stationArry = [BMKPointAnnotation]()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        _mapView = BMKMapView(frame: CGRect(x: 0, y: 0, width:self.view.frame.width, height: self.view.frame.height))
        let point: CGPoint = CGPoint(x: 10, y: 10)
        _mapView?.compassPosition = point
        _mapView?.zoomLevel = 23
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
        _mapView?.showsUserLocation = false//先关闭显示的定位图层
        _mapView?.userTrackingMode = BMKUserTrackingModeNone;//设置定位的状态
        _mapView?.showsUserLocation = true//显示定位图层
        print("进入定位状态")
        _mapView?.showsUserLocation = false
        _mapView?.userTrackingMode = BMKUserTrackingModeFollow
        _mapView?.showsUserLocation = true
        
        print("进入跟随态");
        _mapView!.showsUserLocation = false
        _mapView?.userTrackingMode = BMKUserTrackingModeFollow
        _mapView?.showsUserLocation = true
        
        print("进入罗盘态");
        _mapView?.showsUserLocation = false
        _mapView?.userTrackingMode = BMKUserTrackingModeFollowWithHeading
        _mapView?.showsUserLocation = true
        
        //定义地图区域和中心坐标（
        //使用当前位置
        //var center:CLLocation = locationManager.location.coordinate
        //使用自定义位置
        let center:CLLocationCoordinate2D = CLLocationCoordinate2D(latitude: 32.029171, longitude: 118.788231)
        //设置显示区域
        _mapView?.setCenter(center, animated: true)
        
        //createNewThreadWithNSThread()
        //创建定时任务
        _ = Timer.scheduledTimer(timeInterval: 2,target:self,selector:#selector(NNBaiduMapViewController.timerFireMethod),                                 userInfo:nil,repeats:true)
        
        //request_station()
        
        self.view.addSubview(_mapView!)
        
    }
    // 参数: The timer passes itself as the argument
    func timerFireMethod() {
        
        if( logon == false){
            return
        }
        
        if(process == true && gsocket.isConnected == false){
            gsocket.connect()
        }
        if( gflowstage == 3){
            gflowstage = 0
            self.navigationController!.pushViewController(StopView(), animated: true)
        }
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        Alamofire.request(appUrl + "api/v1/app/cars",method:.get,headers:headers)
            .response { response in

                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    print("Data cars: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    //print(jsondata)
                    let code = jsondata["code"].int32Value
                    if( code == 1 ){
                        self.addcarScreenAnnotation(jsondata: jsondata)
                    }else{
                        print("request cars :\(jsondata["message"].stringValue)")
                        let car_old = [String](self.carArry.keys)
                        for old in car_old{
                            let  rm = self.carArry[old]
                            self._mapView?.removeAnnotation(rm)
                            self.carArry.removeValue(forKey: old)
                        }
                    }
                }
        }
    }
    
    //添加固定car屏幕位置的标注
    func addcarScreenAnnotation(jsondata: JSON) {
        var car_new = [String]()
        let car_list = jsondata["data"].arrayValue
        for car in car_list{
            var gps = car["gps_location"]
            car_new.append(car["license"].stringValue)
            //print("car license : \(car["license"].stringValue)")
            var  Annotation = carArry["\(car["license"].stringValue)"]
            if Annotation == nil {
                Annotation = BMKPointAnnotation()
                Annotation?.coordinate = CLLocationCoordinate2DMake( gps["latitude"].doubleValue, gps["longitude"].doubleValue)
                Annotation?.title = car["license"].stringValue
                Annotation?.subtitle = "car"
                showType = AnnotationShowType.CAR
                _mapView?.addAnnotation(Annotation)
                carArry["\(car["license"].stringValue)"] = Annotation
            }else{
                Annotation?.coordinate = CLLocationCoordinate2DMake(gps["latitude"].doubleValue + 0.114224994017925, gps["longitude"].doubleValue + 0.08625089575709)
            }
        }
        let car_old = [String](carArry.keys)
        for old in car_old{
            var flag:Bool = false
            for new in car_new{
                if new == old{
                    flag = true
                }
            }
            if(flag == false){
                let  rm = carArry[old]
                _mapView?.removeAnnotation(rm)
                carArry.removeValue(forKey: old)
            }
        }
    }
    
    func mesage_show(msg:String){
        let alertVC = UIAlertController(title: "", message: msg, preferredStyle: .alert)
        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
        alertVC.addAction(alertAction)
        self.present(alertVC, animated: true, completion: nil)
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView?.delegate = self
        _mapView?.viewWillAppear()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        locationManager.delegate = nil
        _mapView?.delegate = nil
        _mapView?.viewWillDisappear()
    }
    
    // MARK: - BMKlocationManagerDelegate
    func bmkLocationManager(_ manager: BMKLocationManager, didUpdate heading: CLHeading?) {
        if let tempHeading = heading {
            userLocation.heading = tempHeading
            _mapView?.updateLocationData(userLocation)
        }
    }
    
    func bmkLocationManager(_ manager: BMKLocationManager, didUpdate location: BMKLocation?, orError error: Error?) {
        if let temError = error {
            print("locError:{\(temError.localizedDescription)}")
        }
        if let temLocation = location {
            userLocation.location = temLocation.location;
            _mapView?.updateLocationData(userLocation)
            if(once_flag == 1){
                once_flag = 0
                addLockScreenAnnotation()
                request_station()
            }
        }
    }
    //添加固定屏幕位置的标注
    func addLockScreenAnnotation() {
        if lockedScreenAnnotation == nil {
            lockedScreenAnnotation = BMKPointAnnotation()
            lockedScreenAnnotation?.isLockedToScreen = true
            lockedScreenAnnotation?.screenPointToLock = CGPoint(x: view.frame.size.width/2, y: view.frame.size.height/2)
            lockedScreenAnnotation?.title = "选点位置"
        }
        _mapView?.addAnnotation(lockedScreenAnnotation)
    }

    func request_station(){
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        Alamofire.request(appUrl + "api/v1/app/station",method:.get,headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data station: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    //print(jsondata)
                    print("request_station code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let stations = jsondata["data"]["records"].arrayValue
                        station_json = stations
                        for item in stations {
                            let stationAnnotation = BMKPointAnnotation()
                            if(item["type"].int == 1){
                               self.showType = AnnotationShowType.STATION
                            }else if(item["type"].int == 2){
                               self.showType = AnnotationShowType.PARKING
                            }
                            
                            stationAnnotation.coordinate = CLLocationCoordinate2DMake(item["latitude"].doubleValue + 0.114224994017925, item["longitude"].doubleValue + 0.08625089575709)
                            stationAnnotation.title = "\(item["name"].stringValue)"
                            stationAnnotation.subtitle = " "
                            stationNameArry[item["id"].intValue] = item["name"].stringValue
                            self._mapView?.addAnnotation(stationAnnotation)
                            self.stationArry.append(stationAnnotation)
                        }
                    }else{
                        self.mesage_show(msg:jsondata["message"].stringValue)
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
        return nil
    }
    
    /**
     *当mapView新添加overlay views时，调用此接口
     *@param mapView 地图View
     *@param overlayViews 新添加的overlay views
     */
    func mapView(_ mapView: BMKMapView!, didAddOverlayViews overlayViews: [Any]!) {
        NSLog("didAddOverlayViews")
    }
    /**
     *根据anntation生成对应的View
     *@param mapView 地图View
     *@param annotation 指定的标注
     *@return 生成的标注View
     */
    func mapView(_ mapView: BMKMapView!, viewFor annotation: BMKAnnotation!) -> BMKAnnotationView! {
        print("add annotation: \(String(describing: annotation.title!())) : \(String(describing: annotation.subtitle!()))")
        // 动画标注
        // 普通标注
        let AnnotationViewID = "renameMark"
        var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: AnnotationViewID) as! BMKPinAnnotationView?
        if annotationView == nil {
            annotationView = BMKPinAnnotationView(annotation: annotation, reuseIdentifier: AnnotationViewID)
            if (annotation as! BMKPointAnnotation) == lockedScreenAnnotation {
                // 设置颜色
                annotationView!.pinColor = UInt(BMKPinAnnotationColorGreen)
                // 设置可拖拽
                annotationView!.isDraggable = false
            }else if( self.showType == AnnotationShowType.STATION ){
                self.showType = AnnotationShowType.NONE
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"station1.png")
            }else if( self.showType == AnnotationShowType.PARKING ){
                self.showType = AnnotationShowType.NONE
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"park1.png")
            }
            else if( self.showType == AnnotationShowType.CAR ){
                self.showType = AnnotationShowType.NONE
                // 设置可拖拽
                annotationView!.isDraggable = true
                annotationView!.image = UIImage(named:"car7.png")
            }
        }
        annotationView?.annotation = annotation
        return annotationView
    }
    
    /**
     *当mapView新添加annotation views时，调用此接口
     *@param mapView 地图View
     *@param views 新添加的annotation views
     */
    func mapView(_ mapView: BMKMapView!, didAddAnnotationViews views: [Any]!) {
        NSLog("didAddAnnotationViews")
    }
    
    /**
     *当选中一个annotation views时，调用此接口
     *@param mapView 地图View
     *@param views 选中的annotation views
     */
    func mapView(_ mapView: BMKMapView!, didSelect view: BMKAnnotationView!) {
        if( process == true ){
            return
        }
        let annotaiton = view.annotation
        let title:String = annotaiton?.title!() ?? "default value"
        let subtitle:String = annotaiton?.subtitle!() ?? "default value"
        NSLog("选中了标注 :\(title) :\(subtitle)")
        if(subtitle == "car"){
            license = title
            self.navigationController!.pushViewController(CallCarViewController(), animated: true)
            
            //self.present(CallCarViewController(), animated: true, completion: nil)
            let headers: HTTPHeaders = [
                "Connection" : "keep-alive",
                "Authorization" : "'Bearer \(String(describing: tokenStr))",
                "Content-Type": "application/json"
            ]
            
            Alamofire.request(appUrl + "api/v1/app/cars",method:.get, parameters: ["foo": "bar"],headers:headers)
                .response { response in
                    if let data = response.data, let _ = String(data: data, encoding: .utf8) {

                        let jsondata = JSON(response.data ?? data)
                        if( jsondata["code"].int32Value == 1 ){
                            var jsondata = JSON(response.data ?? data)
                            let car_list = jsondata["data"].arrayValue
                            for car in car_list{
                                if(car["license"].stringValue == license ){
                                    device_id = car["device_id"].stringValue
                                    //跳转新页面
                                    //self.present(CallCarViewController(), animated: true, completion: nil)
                                }
                            }
                        }else{
                            print("request cars :\(jsondata["message"].stringValue)")
                        }
                    }
            }
        }
    }
    /**
     *当取消选中一个annotation views时，调用此接口
     *@param mapView 地图View
     *@param views 取消选中的annotation views
     */
    func mapView(_ mapView: BMKMapView!, didDeselect view: BMKAnnotationView!) {
        NSLog("取消选中标注")
    }
    
    /**
     *拖动annotation view时，若view的状态发生变化，会调用此函数。ios3.2以后支持
     *@param mapView 地图View
     *@param view annotation view
     *@param newState 新状态
     *@param oldState 旧状态
     */
    func mapView(_ mapView: BMKMapView!, annotationView view: BMKAnnotationView!, didChangeDragState newState: UInt, fromOldState oldState: UInt) {
        NSLog("annotation view state change : \(oldState) : \(newState)")
    }
    
    /**
     *当点击annotation view弹出的泡泡时，调用此接口
     *@param mapView 地图View
     *@param view 泡泡所属的annotation view
     */
    func mapView(_ mapView: BMKMapView!, annotationViewForBubble view: BMKAnnotationView!) {
        NSLog("点击了泡泡")
    }
    
    // MARK: - BMKMapViewDelegate
    
    /**
     *点中底图标注后会回调此接口
     *@param mapview 地图View
     *@param mapPoi 标注点信息
     */
    func mapView(_ mapView: BMKMapView!, onClickedMapPoi mapPoi: BMKMapPoi!) {
        //print("您点击了地图标注\(mapPoi.text)，当前经纬度:(\(mapPoi.pt.longitude),\(mapPoi.pt.latitude))，缩放级别:\(mapView.zoomLevel)，旋转角度:\(mapView.rotation)，俯视角度:\(mapView.overlooking)")
    }
    
    /**
     *点中底图空白处会回调此接口
     *@param mapview 地图View
     *@param coordinate 空白处坐标点的经纬度
     */
    func mapView(_ mapView: BMKMapView!, onClickedMapBlank coordinate: CLLocationCoordinate2D) {
        //print("您点击了地图空白处，当前经纬度:(\(coordinate.longitude),\(coordinate.latitude))，缩放级别:\(mapView.zoomLevel)，旋转角度:\(mapView.rotation)，俯视角度:\(mapView.overlooking)")
    
    }
    
    /**
     *双击地图时会回调此接口
     *@param mapview 地图View
     *@param coordinate 返回双击处坐标点的经纬度
     */
    func mapview(_ mapView: BMKMapView!, onDoubleClick coordinate: CLLocationCoordinate2D) {
        //print("您双击了地图，当前经纬度:(\(coordinate.longitude),\(coordinate.latitude))，缩放级别:\(mapView.zoomLevel)，旋转角度:\(mapView.rotation)，俯视角度:\(mapView.overlooking)")
    }
    
    /**
     *长按地图时会回调此接口
     *@param mapview 地图View
     *@param coordinate 返回长按事件坐标点的经纬度
     */
    func mapview(_ mapView: BMKMapView!, onLongClick coordinate: CLLocationCoordinate2D) {
        //print("您长按了地图，当前经纬度:(\(coordinate.longitude),\(coordinate.latitude))，缩放级别:\(mapView.zoomLevel)，旋转角度:\(mapView.rotation)，俯视角度:\(mapView.overlooking)")
    }
    
    /**
     *地图区域改变完成后会调用此接口
     *@param mapview 地图View
     *@param animated 是否动画
     */
    func mapView(_ mapView: BMKMapView!, regionDidChangeAnimated animated: Bool) {
        //print("当前地图区域发生了变化(x = \(mapView.visibleMapRect.origin.x), y = \(mapView.visibleMapRect.origin.y) width = \(mapView.visibleMapRect.size.width), height = \(mapView.visibleMapRect.size.height)) ZoomLevel = \(mapView.zoomLevel), RotateAngle = \(mapView.rotation), OverlookAngle = \(mapView.overlooking)")
    }
}
