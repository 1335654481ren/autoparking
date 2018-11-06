//
//  AnnotationDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/5.
//  Copyright © 2015年 baidu. All rights reserved.
//


class AnnotationDemoViewController: UIViewController, BMKMapViewDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    var circle: BMKCircle?
    var polygon: BMKPolygon?
    var polyline: BMKPolyline?
    var colorfulPolyline: BMKPolyline?
    var arcline: BMKArcline?
    var ground: BMKGroundOverlay?
    var pointAnnotation: BMKPointAnnotation?
    var animatedAnnotation: BMKPointAnnotation?
    var lockedScreenAnnotation: BMKPointAnnotation?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        _mapView.zoomLevel = 12
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
        addOverlayViews()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
    }
    
    // MARK: - IBAction
    
    @IBAction func segmentedValueChangedAction(_ sender: UISegmentedControl) {
        _mapView.removeOverlays(_mapView.overlays)
        _mapView.removeAnnotations(_mapView.annotations)
        switch sender.selectedSegmentIndex {
        case 0://内置覆盖物
            addOverlayViews()
        case 1://标注
            addPointAnnotation()
            addAnimatedAnnotation()
            addLockScreenAnnotation()
        case 2://图片图层
            addGroundOverlay()
        default:
            break
        }
    }
    
    // MARK: - 
    
    //添加内置覆盖物
    func addOverlayViews() {
        // 添加圆形覆盖物
        if circle == nil {
            circle = BMKCircle(center: CLLocationCoordinate2DMake(39.915, 116.404), radius: 5000)
        }
        _mapView.add(circle)
        
        // 添加多边形覆盖物
        if polygon == nil {
            var coords = [CLLocationCoordinate2D]()
            coords.append(CLLocationCoordinate2DMake(39.915, 116.404))
            coords.append(CLLocationCoordinate2DMake(39.815, 116.404))
            coords.append(CLLocationCoordinate2DMake(39.815, 116.504))
            coords.append(CLLocationCoordinate2DMake(39.915, 116.504))
            polygon = BMKPolygon(coordinates: &coords, count: UInt(coords.count))
        }
        _mapView.add(polygon)
        
        // 添加折线覆盖物
        if polyline == nil {
            var coords = [
                CLLocationCoordinate2DMake(39.895, 116.354),
                CLLocationCoordinate2DMake(39.815, 116.304)]
            polyline = BMKPolyline(coordinates: &coords, count: 2)
        }
        _mapView.add(polyline)
        
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
        _mapView.add(colorfulPolyline)
        
        // 添加圆弧覆盖物
        if arcline == nil {
            var coords = [
                CLLocationCoordinate2DMake(40.065, 116.124),
                CLLocationCoordinate2DMake(40.125, 116.304),
                CLLocationCoordinate2DMake(40.065, 116.404)]
            arcline = BMKArcline(coordinates: &coords)
        }
        _mapView.add(arcline)
    }
    
    //添加标注
    func addPointAnnotation() {
        if pointAnnotation == nil {
            pointAnnotation = BMKPointAnnotation()
            pointAnnotation?.coordinate = CLLocationCoordinate2DMake(39.915, 116.404)
            pointAnnotation?.title = "我是pointAnnotation"
            pointAnnotation?.subtitle = "此Annotation可拖拽!"
        }
        _mapView.addAnnotation(pointAnnotation)
    }
    //添加固定屏幕位置的标注
    func addLockScreenAnnotation() {
        if lockedScreenAnnotation == nil {
            lockedScreenAnnotation = BMKPointAnnotation()
            lockedScreenAnnotation?.isLockedToScreen = true
            lockedScreenAnnotation?.screenPointToLock = CGPoint(x: 100, y: 100)
            lockedScreenAnnotation?.title = "我是固定屏幕的标注"
        }
        _mapView.addAnnotation(lockedScreenAnnotation)
    }
    
    // 添加动画Annotation
    func addAnimatedAnnotation() {
        if animatedAnnotation == nil {
            animatedAnnotation = BMKPointAnnotation()
            animatedAnnotation?.coordinate = CLLocationCoordinate2DMake(40.115, 116.404)
            animatedAnnotation?.title = "我是动画Annotation"
        }
        _mapView.addAnnotation(animatedAnnotation)
    }
    
    //添加图片图层覆盖物
    func addGroundOverlay() {
        if ground == nil {
            var bound = BMKCoordinateBounds()
            bound.southWest = CLLocationCoordinate2DMake(39.910, 116.370)
            bound.northEast = CLLocationCoordinate2DMake(39.950, 116.430)
            ground = BMKGroundOverlay(bounds: bound, icon: UIImage(named: "test.png"))
        }
        _mapView.add(ground)
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
                annotationView!.pinColor = UInt(BMKPinAnnotationColorPurple)
                // 设置可拖拽
                annotationView!.isDraggable = false
            } else {
                annotationView!.isDraggable = true
            }
            // 从天上掉下的动画
            annotationView!.animatesDrop = true
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
        NSLog("选中了标注")
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
    
}
