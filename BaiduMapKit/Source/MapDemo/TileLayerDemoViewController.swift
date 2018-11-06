//
//  TileLayerDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/9.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class LocalTileLayer: BMKSyncTileLayer {
    override func tileFor(x: Int, y: Int, zoom: Int) -> UIImage! {
        let image = UIImage(named: "\(zoom)_\(x)_\(y).jpg")
        return image;
    }
}

class TileLayerDemoViewController: UIViewController, BMKMapViewDelegate {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        _mapView.zoomLevel = 16
        _mapView.centerCoordinate = CLLocationCoordinate2DMake(39.924, 116.400)
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

    // MARK: - IBAction
    
    @IBAction func changeTileStatus(_ sender: UISegmentedControl) {
        _mapView.removeOverlays(_mapView.overlays)
        if sender.selectedSegmentIndex == 1 {//本地
            addLocalTile()
        } else if sender.selectedSegmentIndex == 2 {//在线
            addUrlTile()
        } else {
            _mapView.mapType = BMKMapType.standard
        }
    }
    //添加本地瓦片图
    func addLocalTile() {
        //限制地图显示范围
        _mapView.maxZoomLevel = 17.4
        _mapView.minZoomLevel = 16
        let center = CLLocationCoordinate2DMake(39.924257, 116.403263)
        let span = BMKCoordinateSpanMake(0.013142, 0.011678)
        _mapView.limitMapRegion = BMKCoordinateRegionMake(center, span)
        _mapView.isOverlookEnabled = false
        _mapView.isRotateEnabled = false//禁用旋转手势
        _mapView.mapType = BMKMapType.none
        
        let localTileLayer = LocalTileLayer()
        localTileLayer.visibleMapRect = BMKMapRectMake(32995300, 35855667, 1300, 1900)
        localTileLayer.maxZoom = 18
        localTileLayer.minZoom = 16
        _mapView.add(localTileLayer)
    }
    //添加在线瓦片图
    func addUrlTile() {
        //限制地图显示范围
        _mapView.maxZoomLevel = 18
        _mapView.minZoomLevel = 16
        let center = CLLocationCoordinate2DMake(39.924257, 116.403263)
        let span = BMKCoordinateSpanMake(0.038325, 0.028045)
        _mapView.limitMapRegion = BMKCoordinateRegionMake(center, span)
        _mapView.isOverlookEnabled = false
        _mapView.isRotateEnabled = false//禁用旋转手势
        _mapView.mapType = BMKMapType.none
        
        let urlTileLayer = BMKURLTileLayer(urlTemplate: "http://api0.map.bdimg.com/customimage/tile?&x={x}&y={y}&z={z}&udt=20150601&customid=light")
        urlTileLayer?.visibleMapRect = BMKMapRectMake(32994258, 35853667, 3122, 5541)
        urlTileLayer?.maxZoom = 18
        urlTileLayer?.minZoom = 16
        _mapView.add(urlTileLayer)
    }
    
    // MARK: - BMKMapViewDelegate
    /**
    *根据overlay生成对应的View
    *@param mapView 地图View
    *@param overlay 指定的overlay
    *@return 生成的覆盖物View
    */
    func mapView(_ mapView: BMKMapView!, viewFor overlay: BMKOverlay!) -> BMKOverlayView! {
        if overlay is BMKTileLayer {
            return BMKTileLayerView()
        }
        return nil;
    }
}
