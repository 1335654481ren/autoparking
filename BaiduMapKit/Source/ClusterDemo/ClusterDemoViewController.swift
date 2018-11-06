//
//  ClusterDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/9.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

/*
*点聚合Annotation
*/
class ClusterAnnotation : BMKPointAnnotation {
    var size: UInt = 0
}

/*
*点聚合AnnotationView
*/
class ClusterAnnotationView : BMKPinAnnotationView {
    var label: UILabel?
    
    override init(annotation: BMKAnnotation!, reuseIdentifier: String!) {
        super.init(annotation: annotation, reuseIdentifier: reuseIdentifier)
        
        self.bounds = CGRect(x: 0, y: 0, width: 22, height: 22)
        label = UILabel(frame: CGRect(x: 0, y: 0, width: 22, height: 22))
        label?.textColor = UIColor.white
        label?.font = UIFont.systemFont(ofSize: 11)
        label?.textAlignment = NSTextAlignment.center
        self.addSubview(label!)
        self.alpha = 0.85;
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    internal func setSize(_ size :UInt) {
        if size == 1 {
            label?.isHidden = true
            self.pinColor = UInt(BMKPinAnnotationColorRed)
            return;
        }
        label?.isHidden = false
        if size > 20 {
            label?.backgroundColor = UIColor.red
        } else if size > 10 {
            label?.backgroundColor = UIColor.purple
        } else if size > 5 {
            label?.backgroundColor = UIColor.blue
        } else {
            label?.backgroundColor = UIColor.green
        }
        label?.text = "\(size)"
    }

}


class ClusterDemoViewController: UIViewController, BMKMapViewDelegate  {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    //点聚合管理类
    var _clusterManager: BMKClusterManager!
    //聚合级别
    var _clusterZoom: Int! = 0
    //点聚合缓存标注
    var _clusterCaches = Array(repeating: [ClusterAnnotation](), count: 19)
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
         //点聚合管理类
        _clusterManager = BMKClusterManager()
        let coor = CLLocationCoordinate2DMake(39.915, 116.404)
        //向点聚合管理类中添加标注
        for _ in 0..<20 {
            let lat = Double(arc4random() % 100) / 1000.0
            let lon = Double(arc4random() % 100) / 1000.0
            let clusterItem = BMKClusterItem()
            clusterItem.coor = CLLocationCoordinate2DMake(coor.latitude + lat, coor.longitude + lon)
            _clusterManager.add(clusterItem)
        }
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
    }

    // MARK: - 
    //更新聚合状态
    func updateClusters() {
        _clusterZoom = Int(_mapView.zoomLevel)
        var clusters = _clusterCaches[_clusterZoom - 3]
        if clusters.count > 0 {
            _mapView.removeAnnotations(_mapView.annotations)
            _mapView.addAnnotations(clusters)
        } else {
            DispatchQueue.global(qos: .default).async(execute: { () -> Void in
                ///获取聚合后的标注
                if let array = self._clusterManager.getClusters(Float(self._clusterZoom)) as? [BMKCluster] {
                    DispatchQueue.main.async(execute: { () -> Void in
                        for item in array {
                            let annotation = ClusterAnnotation()
                            annotation.coordinate = item.coordinate
                            annotation.size = item.size
                            annotation.title = "我是\(item.size)个"
                            clusters.append(annotation)
                        }
                        self._mapView.removeAnnotations(self._mapView.annotations)
                        self._mapView.addAnnotations(clusters)
                    })
                }
            })
        }
    }
    
    // MARK: - BMKMapViewDelegate
    
    // 根据anntation生成对应的View
    func mapView(_ mapView: BMKMapView!, viewFor annotation: BMKAnnotation!) -> BMKAnnotationView! {
        if let cluster = annotation as? ClusterAnnotation {
            let annotationView = ClusterAnnotationView(annotation: cluster, reuseIdentifier: "clustermark")
            annotationView.setSize(cluster.size)
            annotationView.annotation = cluster
            return annotationView
        }
        return nil
    }
   
    
    /**
    *当点击annotation view弹出的泡泡时，调用此接口
    *@param mapView 地图View
    *@param view 泡泡所属的annotation view
    */
    func mapView(_ mapView: BMKMapView!, annotationViewForBubble view: BMKAnnotationView!) {
        if view is ClusterAnnotationView {
            if let cluster = view.annotation as? ClusterAnnotation {
                if cluster.size > 1 {
                    _mapView.centerCoordinate = cluster.coordinate
                    _mapView.zoomIn()
                }
            }
        }
    }
    
    /**
    *地图初始化完毕时会调用此接口
    *@param mapview 地图View
    */
    func mapViewDidFinishLoading(_ mapView: BMKMapView!) {
        updateClusters()
    }
    
    /**
    *地图渲染每一帧画面过程中，以及每次需要重绘地图时（例如添加覆盖物）都会调用此接口
    *@param mapview 地图View
    *@param status 此时地图的状态
    */
    func mapView(_ mapView: BMKMapView!, onDrawMapFrame status: BMKMapStatus!) {
        if (_clusterZoom != 0 && _clusterZoom != Int(mapView.zoomLevel)) {
            updateClusters()
        }
    }

}
