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
    var cur_station: String!
    var des_station: String!
    var car_id: String!
    var user_id:String = "USER-0001"
    var circle: BMKCircle?
    var polygon: BMKPolygon?
    var polyline: BMKPolyline?
    var colorfulPolyline: BMKPolyline?
    var arcline: BMKArcline?
    var ground: BMKGroundOverlay?
    var pointAnnotation: BMKPointAnnotation?
    var animatedAnnotation: BMKPointAnnotation?
    var lockedScreenAnnotation: BMKPointAnnotation?
    var carlockedScreenAnnotation: BMKPointAnnotation?
    var station1LockScreenAnnotation: BMKPointAnnotation?
    var station2LockScreenAnnotation: BMKPointAnnotation?
    var station3LockScreenAnnotation: BMKPointAnnotation?
    var station4LockScreenAnnotation: BMKPointAnnotation?
    var once_flag = 1
    //定义一个UIButton按钮
    let station1_button:UIButton = UIButton(type: .system)
    let station2_button:UIButton = UIButton(type: .system)
    let parking1_button:UIButton = UIButton(type: .system)
    let parking2_button:UIButton = UIButton(type: .system)
    
    var baidu_index = 0
    
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
        
        set_button()
        
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
                    print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    //print(jsondata)
                    if( jsondata["code"].int32Value == 1 ){
                        self.addcarScreenAnnotation(jsondata: jsondata)
                    }else{
                        print("request cars :\(jsondata["message"].stringValue)")
                    }
                }
        }
    }
    
    //添加固定car屏幕位置的标注
    func addcarScreenAnnotation(jsondata: JSON) {
        
        let car_list = jsondata["data"].arrayValue
        for car in car_list{
            var gps = car["gps_location"]
            //print("car license : \(car["license"].stringValue)")
            if carlockedScreenAnnotation == nil {
                carlockedScreenAnnotation = BMKPointAnnotation()
                carlockedScreenAnnotation?.coordinate = CLLocationCoordinate2DMake( gps["latitude"].doubleValue, gps["longitude"].doubleValue)
                carlockedScreenAnnotation?.title = car["license"].stringValue
                carlockedScreenAnnotation?.subtitle = "car"
                _mapView?.addAnnotation(carlockedScreenAnnotation)
            }else{
                carlockedScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(gps["latitude"].doubleValue + 0.114224994017925, gps["longitude"].doubleValue + 0.08625089575709)
            }
        }
    }
    
    func addTESTcarScreenAnnotation() {
        
            if carlockedScreenAnnotation == nil {
                carlockedScreenAnnotation = BMKPointAnnotation()
                carlockedScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(40.065, 116.124)
                carlockedScreenAnnotation?.title = "EQ0000001"
                carlockedScreenAnnotation?.subtitle = "car"
                _mapView?.addAnnotation(carlockedScreenAnnotation)
            }
    }
    
    func set_button(){
        
        //button的大小位置
        station1_button.frame = CGRect(x: 30, y: 20, width: 100, height: 30)
        //设置按钮显示的标题
        station1_button.setTitle("Call", for: .normal)
        //设置按钮的背景颜色
        station1_button.backgroundColor = UIColor.orange
        //设置点击响应事件
        station1_button.addTarget(self, action:#selector(station1_button_service(_:)), for:.touchUpInside)
        
        //button的大小位置
        station2_button.frame = CGRect(x: 30, y: 20, width: 100, height: 30)
        //设置按钮显示的标题
        station2_button.setTitle("Call", for: .normal)
        //设置按钮的背景颜色
        station2_button.backgroundColor = UIColor.orange
        //设置点击响应事件
        station2_button.addTarget(self, action:#selector(station2_button_service(_:)), for:.touchUpInside)
        
        //button的大小位置
        parking1_button.frame = CGRect(x: 30, y: 20, width: 100, height: 30)
        //设置按钮显示的标题
        parking1_button.setTitle("Park", for: .normal)
        //设置按钮的背景颜色
        parking1_button.backgroundColor = UIColor.green
        //设置点击响应事件
        parking1_button.addTarget(self, action:#selector(parking1_button_service(_:)), for:.touchUpInside)
        
        //button的大小位置
        parking2_button.frame = CGRect(x: 30, y: 20, width: 100, height: 30)
        //设置按钮显示的标题
        parking2_button.setTitle("Park", for: .normal)
        //设置按钮的背景颜色
        parking2_button.backgroundColor = UIColor.green
        //设置点击响应事件
        parking2_button.addTarget(self, action:#selector(parking2_button_service(_:)), for:.touchUpInside)
        
    }
    func mesage_show(msg:String){
        let alertVC = UIAlertController(title: "", message: msg, preferredStyle: .alert)
        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
        alertVC.addAction(alertAction)
        self.present(alertVC, animated: true, completion: nil)
    }
    
    func station1_button_service(_ button:UIButton) {
        if(process == true){
            self.mesage_show(msg: "The car is on the way")
            return
        }
        var root = JSON()
        var station = JSON()
        station["station_num"].int32 = 2
        station["x"].double = 11.11
        station["y"].double = 22.22
        station["z"].double = 33.33
        root["station"].object = station
        root["device_id"].string  = device_id
        root["command"].int32 = 1
        let jsonstr = root.rawString()

        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        let manager = Alamofire.SessionManager.default
        manager.session.configuration.timeoutIntervalForRequest = 3
        
        manager.request(appUrl + "api/v1/app/callcar",method:.post, parameters: ["body": jsonstr ?? "default velua"], headers:headers)
            .response { response in

                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        var car = jsondata["data"]
                        route = car["route"].arrayObject as! [Int]
                        self.baidu_index = 1
                        self.mesage_show(msg: "The car is arriving!")
                    }else{
                        self.mesage_show(msg: jsondata["Error message"].stringValue)
                    }
                }
        }
    }
    
    func station2_button_service(_ button:UIButton) {
        print("我摁下1了这个按钮")
        var root = JSON()
        var station = JSON()
        station["station_num"].int32 = 1
        station["x"].double = 11.11
        station["y"].double = 22.22
        station["z"].double = 33.33
        root["station"].object = station
        root["device_id"].string  = license
        root["command"].int32 = 1
        let jsonstr = root.rawString()
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        let manager = Alamofire.SessionManager.default
        manager.session.configuration.timeoutIntervalForRequest = 3
        
        manager.request(appUrl + "api/v1/app/callcar",method:.post, parameters: ["body": jsonstr ?? "default velua"], headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let alertVC = UIAlertController(title: "", message: "叫车成功", preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                        
                        var car = jsondata["data"]
                        route = car["route"].arrayObject as! [Int]
                        
                        self.baidu_index = 1
                    }else{
                        let alertVC = UIAlertController(title: "", message: jsondata["Error message"].stringValue, preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                    }
                }
        }
    }
    func parking1_button_service(_ button:UIButton) {
        print("我摁下1了这个按钮")
        var root = JSON()
        var station = JSON()
        station["station_num"].int32 = 3
        station["x"].double = 11.11
        station["y"].double = 22.22
        station["z"].double = 33.33
        root["station"].object = station
        root["device_id"].string  = license
        root["command"].int32 = 1
        let jsonstr = root.rawString()
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        let manager = Alamofire.SessionManager.default
        manager.session.configuration.timeoutIntervalForRequest = 3
        
        manager.request(appUrl + "api/v1/app/callcar",method:.post, parameters: ["body": jsonstr ?? "default velua"], headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let alertVC = UIAlertController(title: "", message: "叫车成功", preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                        
                        var car = jsondata["data"]
                        route = car["route"].arrayObject as! [Int]
                        
                        self.baidu_index = 1
                    }else{
                        let alertVC = UIAlertController(title: "", message: jsondata["Error message"].stringValue, preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                    }
                }
        }
    }
    
    func parking2_button_service(_ button:UIButton) {
        print("我摁下1了这个按钮")
        var root = JSON()
        var station = JSON()
        station["station_num"].int32 = 4
        station["x"].double = 11.11
        station["y"].double = 22.22
        station["z"].double = 33.33
        root["station"].object = station
        root["device_id"].string  = license
        root["command"].int32 = 1
        let jsonstr = root.rawString()
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        let manager = Alamofire.SessionManager.default
        manager.session.configuration.timeoutIntervalForRequest = 3
        
        manager.request(appUrl + "api/v1/app/callcar",method:.post, parameters: ["body": jsonstr ?? "default velua"], headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let alertVC = UIAlertController(title: "", message: "叫车成功", preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                        
                        var car = jsondata["data"]
                        route = car["route"].arrayObject as! [Int]
                        
                        self.baidu_index = 1
                    }else{
                        let alertVC = UIAlertController(title: "", message: jsondata["Error message"].stringValue, preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                    }
                }
        }
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
    
    // MARK: Write Text Action
    func websoket_send(data:String) {
        var send_msg = JSON()
        send_msg["user_id"] = "USER_00001"
        if(socket.isConnected){
            socket.write(string:data)
        }
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
                //addPointAnnotation()
                //addAnimatedAnnotation()
                addLockScreenAnnotation()
                addstationLockScreenAnnotation()
                //addTESTcarScreenAnnotation()
            }
        }
    }
    // MARK: -
    //添加内置覆盖物
    func addOverlayViews() {
        // 添加圆形覆盖物
        if circle == nil {
            circle = BMKCircle(center: CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude), radius: 5000)
        }
        _mapView?.add(circle)
        
        // 添加多边形覆盖物
        if polygon == nil {
            var coords = [CLLocationCoordinate2D]()
            coords.append(CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude))
            coords.append(CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude))
            coords.append(CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude))
            coords.append(CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude))
            polygon = BMKPolygon(coordinates: &coords, count: UInt(coords.count))
        }
        _mapView?.add(polygon)
        
        // 添加折线覆盖物
        if polyline == nil {
            var coords = [
                CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude),
                CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude)]
            polyline = BMKPolyline(coordinates: &coords, count: 2)
        }
        _mapView?.add(polyline)
        
        // 添加折线(分段颜色绘制)覆盖物
        if colorfulPolyline == nil {
            var coords = [
                CLLocationCoordinate2DMake(39.965, 116.404),
                CLLocationCoordinate2DMake(39.925, 116.454),
                CLLocationCoordinate2DMake(39.955, 116.494),
                CLLocationCoordinate2DMake(39.905, 116.554),
                CLLocationCoordinate2DMake(39.965, 116.604)]
            //构建分段颜色索引数组
            let colorIndex = [2, 0, 1, 2]
            //构建BMKPolyline,使用分段颜色索引，其对应的BMKPolylineView必须设置colors属性
            colorfulPolyline = BMKPolyline(coordinates: &coords, count: 5, textureIndex: colorIndex)
        }
        _mapView?.add(colorfulPolyline)
        
        // 添加圆弧覆盖物
        if arcline == nil {
            var coords = [
                CLLocationCoordinate2DMake(40.065, 116.124),
                CLLocationCoordinate2DMake(40.125, 116.304),
                CLLocationCoordinate2DMake(40.065, 116.404)]
            arcline = BMKArcline(coordinates: &coords)
        }
        _mapView?.add(arcline)
    }
    
    //添加标注
    func addPointAnnotation() {
        if pointAnnotation == nil {
            pointAnnotation = BMKPointAnnotation()
            pointAnnotation?.coordinate = CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude)
            pointAnnotation?.title = "我是pointAnnotation"
            pointAnnotation?.subtitle = "可拖拽"
        }
        _mapView?.addAnnotation(pointAnnotation)
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
    //添加固定car屏幕位置的标注
    func addCarLockScreenAnnotation() {
        if carlockedScreenAnnotation == nil {
            carlockedScreenAnnotation = BMKPointAnnotation()
            carlockedScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude)
            carlockedScreenAnnotation?.title = "选择车辆"
            carlockedScreenAnnotation?.subtitle = "可用"
        }
        _mapView?.addAnnotation(carlockedScreenAnnotation)
    }
    //添加固定car屏幕位置的标注
    func addstationLockScreenAnnotation() {
        //1. Ball room; 2. Garage; 3. Lobby; 4. Paking lot
        if station1LockScreenAnnotation == nil {
            station1LockScreenAnnotation = BMKPointAnnotation()
            station1LockScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(31.278715730994175, 121.19748610519817)
            station1LockScreenAnnotation?.title = "Station2"
            station1LockScreenAnnotation?.subtitle = "Position:Lobby"
            _mapView?.addAnnotation(station1LockScreenAnnotation)
            station2LockScreenAnnotation = BMKPointAnnotation()
            station2LockScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(31.278809374303123,121.19909017060738)
            station2LockScreenAnnotation?.title = "Station1"
            station2LockScreenAnnotation?.subtitle = "Position:Ball room"
            _mapView?.addAnnotation(station2LockScreenAnnotation)
            station3LockScreenAnnotation = BMKPointAnnotation()
            station3LockScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(31.279081692210433,121.19732607195611)
            station3LockScreenAnnotation?.title = "Park Lot 1"
            station3LockScreenAnnotation?.subtitle = "Position:Garage"
            _mapView?.addAnnotation(station3LockScreenAnnotation)
            
            station4LockScreenAnnotation = BMKPointAnnotation()
            station4LockScreenAnnotation?.coordinate = CLLocationCoordinate2DMake(31.278975411617925,121.19919755525709)
            //31.1647504176 ,121.1129466595
            // 0.114224994017925 ,0.08625089575709
            station4LockScreenAnnotation?.title = "Park Lot 2"
            station4LockScreenAnnotation?.subtitle = "Position:Paking lot"
            _mapView?.addAnnotation(station4LockScreenAnnotation)
//2 121.19748610519817,31.278715730994175
        }
    }
    // 添加动画Annotation
    func addAnimatedAnnotation() {
        if animatedAnnotation == nil {
            animatedAnnotation = BMKPointAnnotation()
            animatedAnnotation?.coordinate = CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude, userLocation.location.coordinate.longitude)
            animatedAnnotation?.title = "我是动画Annotation"
        }
        _mapView?.addAnnotation(animatedAnnotation)
    }
    
    //添加图片图层覆盖物
    func addGroundOverlay() {
        if ground == nil {
            var bound = BMKCoordinateBounds()
            bound.southWest = CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude - 0.5, userLocation.location.coordinate.longitude - 0.5)
            bound.northEast = CLLocationCoordinate2DMake(userLocation.location.coordinate.latitude - 0.5, userLocation.location.coordinate.longitude - 0.5)
            ground = BMKGroundOverlay(bounds: bound, icon: UIImage(named: "car.png"))
        }
        print("加载了图片")
        _mapView?.add(ground)
    }
    
    // MARK: - BMKMapViewDelegate
    
    /**
     *根据overlay生成对应的View
     *@param mapView 地图View
     *@param overlay 指定的overlay
     *@return 生成的覆盖物View
     */
    func mapView(_ mapView: BMKMapView!, viewFor overlay: BMKOverlay!) -> BMKOverlayView! {
        
        if (overlay as? BMKCircle) != nil {
            let circleView = BMKCircleView(overlay: overlay)
            circleView?.fillColor = UIColor(red: 1, green: 0, blue: 0, alpha: 0.5)
            circleView?.strokeColor = UIColor(red: 0, green: 0, blue: 1, alpha: 0.5)
            circleView?.lineWidth = 5
            
            return circleView
        }
        
        if (overlay as? BMKPolygon) != nil {
            let polygonView = BMKPolygonView(overlay: overlay)
            polygonView?.strokeColor = UIColor(red: 0, green: 0, blue: 0.5, alpha: 1)
            polygonView?.fillColor = UIColor(red: 0, green: 1, blue: 1, alpha: 0.2)
            polygonView?.lineWidth = 2
            polygonView?.lineDash = true
            return polygonView
        }
        
        if let overlayTemp = overlay as? BMKPolyline {
            let polylineView = BMKPolylineView(overlay: overlay)
            if overlayTemp == polyline {
                polylineView?.strokeColor = UIColor(red: 0, green: 1, blue: 0, alpha: 1)
                polylineView?.lineWidth = 10
                polylineView?.loadStrokeTextureImage(UIImage(named: "texture_arrow.png"))
            } else if overlayTemp == colorfulPolyline {
                polylineView?.lineWidth = 5
                /// 使用分段颜色绘制时，必须设置（内容必须为UIColor）
                polylineView?.colors = [UIColor(red: 0, green: 1, blue: 0, alpha: 1),
                                        UIColor(red: 1, green: 0, blue: 0, alpha: 1),
                                        UIColor(red: 1, green: 1, blue: 0, alpha: 1)]
            }
            return polylineView
        }
        
        if (overlay as? BMKGroundOverlay) != nil {
            let groundView = BMKGroundOverlayView(overlay: overlay)
            return groundView
        }
        
        if (overlay as? BMKArcline) != nil {
            let arclineView = BMKArclineView(overlay: overlay)
            arclineView?.strokeColor = UIColor(red: 0, green: 0, blue: 1, alpha: 1)
            arclineView?.lineDash = true
            arclineView?.lineWidth = 6
            
            return arclineView
        }
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
        if (annotation as! BMKPointAnnotation) == animatedAnnotation {
            let AnnotationViewID = "AnimatedAnnotation"
            let annotationView = AnimatedAnnotationView(annotation: annotation, reuseIdentifier: AnnotationViewID)
            
            var images = Array(repeating: UIImage(), count: 3)
            for i in 1...3 {
                let image = UIImage(named: "poi_\(i).png")
                images[i-1] = image!
            }
            annotationView.setImages(images)
            return annotationView
        }
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
            }else if (annotation as! BMKPointAnnotation) == carlockedScreenAnnotation{
                // 设置可拖拽
                annotationView!.isDraggable = true
                annotationView!.image = UIImage(named:"car7.png")
            }else if (annotation as! BMKPointAnnotation) == station1LockScreenAnnotation{
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"station1.png")
                //if( annotation.title == "站点1"){
                    
                //}
                annotationView?.leftCalloutAccessoryView = station1_button
            }else if (annotation as! BMKPointAnnotation) == station2LockScreenAnnotation{
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"station1.png")
                annotationView?.leftCalloutAccessoryView = station2_button
            }else if (annotation as! BMKPointAnnotation) == station3LockScreenAnnotation{
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"park1.png")
                annotationView?.leftCalloutAccessoryView = parking1_button
            }else if (annotation as! BMKPointAnnotation) == station4LockScreenAnnotation{
                // 设置不可拖拽
                annotationView!.isDraggable = false
                annotationView!.image = UIImage(named:"park1.png")
                annotationView?.leftCalloutAccessoryView = parking2_button
            } else {
                annotationView!.isDraggable = true
            }
            // 从天上掉下的动画
            //annotationView!.animatesDrop = true
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
    
    func createNewThreadWithNSThread()
    {
        //开启一条后台线程，自动启动线程，但无法获得线程对象
        //self.performSelector(inBackground: #selector(NNBaiduMapViewController.thread_run), with: nil);
        //分离出一条子线程，自动启动线程，但无法获得线程对象
        print("create thread")
        //Thread.detachNewThreadSelector(#selector(NNBaiduMapViewController.thread_run), toTarget: self, with: nil)
        //1.创建线程
        let thread = Thread.init(target: self, selector:#selector(NNBaiduMapViewController.thread_run), object: nil)
        //设置线程的名称
        thread.name = "sync"
        //2.启动线程
        thread.start()
    }
    
    func thread_run()
    {
        //获得当前执行run方法的线程
        let thread = Thread.current
        
        let headers: HTTPHeaders = [
            "Host" : "ihome.ngrok.xiaomiqiu.cn",
            "Connection" : "keep-alive",
            "Authorization" : "Basic dXNlcjoxMjM0NQ==",
            "Accept": "application/json"
        ]
        
        while true {
            Alamofire.request("http://ihome.ngrok.xiaomiqiu.cn/asp/car_info.asp",method:.get, parameters: ["foo": "bar"],headers:headers)
                .response { response in
                    //print("Request: \(response.request)")
                    //print("Response: \(response.response)")
                    //print("Error: \(response.error)")
                    if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                        //print("Data: \(data)")
                        //转成JSON对象
                        var jsondata = JSON(response.data ?? data)
                        //print("status ::\(jsondata["status"].stringValue)")
                        //print("car_id ::\(jsondata["array"][0]["car_id"].stringValue)")
                        //var car_list = jsondata[]
                        let car_list = jsondata["data"].arrayValue
                        for car in car_list{
                            //print("\(car["car_id"].stringValue)")
                            device_id = car["device_id"].stringValue
                        }
                    }
            }
        }
    }
}
