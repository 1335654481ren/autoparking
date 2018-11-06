//
//  RootViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/2.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class RootViewController: UIViewController, UITableViewDataSource, UITableViewDelegate {
    let _demoNameArray = ["地图功能-MapViewBaseDemo",
        "多地图使用功能-MultiMapViewDemo",
        "图层展示功能-MapViewDemo",
        "地图操作功能-MapViewControlDemo",
        "UI控制功能-MapViewUISettingDemo",
        "定位功能-LocationDemo",
        "覆盖物功能-AnnotationDemo",
        "自定义绘制-CustomOverlayDemo",
        "POI搜索功能-PoiSearchDemo",
        "地理编码功能-GeocodeDemo",
        "路径规划功能-RouteSearchDemo",
        "公交线路查询-BusLineSearch",
        "行政区域检索功能-DistrictSearchDemo",
        "离线地图功能-OfflineDemo",
        "热力图功能-HeatMapDemo",
        "短串分享功能-ShortUrlShareDemo",
        "云检索功能-CloudSearchDemo",
        "调启地图客户端-OpenMapDemo",
        "OpenGL绘制功能-OpenGLDemo",
        "收藏夹功能-FavoritesDemo",
        "点聚合功能-ClusterDemo",
        "tileLayer功能-TileLayerDemo",
        "室内功能-IndoorDemo"]
    
    let _viewControllerTitleArray = ["地图功能",
        "多地图使用功能",
        "图层展示功能",
        "地图操作功能",
        "UI控制功能",
        "定位功能",
        "覆盖物功能",
        "自定义绘制",
        "POI搜索功能",
        "地理编码功能",
        "路径规划功能",
        "公交线路查询",
        "行政区域检索功能",
        "离线地图功能",
        "热力图功能",
        "短串分享功能",
        "云检索功能",
        "调启百度地图客户端",
        "OpenGL绘制功能",
        "收藏夹功能",
        "点聚合功能",
        "tileLayer功能",
        "室内功能"]
    
    let _viewControllerArray = ["MapViewBaseDemoViewController",
        "MultiMapViewDemoViewController",
        "MapViewDemoViewController",
        "MapViewControlDemoViewController",
        "MapViewUISettingDemoViewController",
        "LocationDemoViewController",
        "AnnotationDemoViewController",
        "CustomOverlayDemoViewController",
        "PoiSearchDemoViewController",
        "GeocodeDemoViewController",
        "RouteSearchDemoViewController",
        "BusLineSearchViewController",
        "DistrictSearchDemoViewController",
        "OfflineDemoViewController",
        "HeatMapDemoViewController",
        "ShortUrlShareDemoViewController",
        "CloudSearchDemoViewController",
        "OpenBaiduMapDemoViewController",
        "OpenGLDemoViewController",
        "FavoritesDemoViewController",
        "ClusterDemoViewController",
        "TileLayerDemoViewController",
        "IndoorDemoViewController"]
    
    //自定义地图样式
    func customMapStyle() {
        //设置自定义地图样式，会影响所有地图实例
        //注：必须在BMKMapView对象初始化之前调用
        let path = Bundle.main.path(forResource: "custom_config_清新蓝", ofType: "")//个性化地图样式文件路径
        BMKMapView.customMapStyle(path)
        BMKMapView.enableCustomMapStyle(false)//默认关闭
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        customMapStyle()//自定义地图样式
        
        self.title = "欢迎使用百度地图iOS SDK \(BMKGetMapApiVersion()!)"
        let backBarItem = UIBarButtonItem()
        backBarItem.title = "返回"
        self.navigationItem.backBarButtonItem = backBarItem
        
        print("base     component version: \(BMKGetMapApiBaseComponentVersion()!)")
        print("map      component version: \(BMKGetMapApiMapComponentVersion()!)")
        print("search   component version: \(BMKGetMapApiSearchComponentVersion()!)")
        print("cloud    component version: \(BMKGetMapApiCloudComponentVersion()!)")
        print("utils    component version: \(BMKGetMapApiUtilsComponentVersion()!)")
    }
    
    
    // MARK: - UITableViewDataSource, UITableViewDelegate
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return _demoNameArray.count
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cellIdentifier = "RootTableViewCell"
        var cell = tableView.dequeueReusableCell(withIdentifier: cellIdentifier)
        if cell == nil {
            cell = UITableViewCell(style: UITableViewCellStyle.default, reuseIdentifier: cellIdentifier)
        }
        
        cell?.textLabel?.text = _demoNameArray[indexPath.row]
        return cell!
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        let vc = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: _viewControllerArray[indexPath.row])
        vc.title = _viewControllerTitleArray[indexPath.row]
        self.navigationController?.pushViewController(vc, animated: true)
    }
}
