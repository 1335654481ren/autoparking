//
//  OfflineDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/9.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class OfflineDemoViewController: UIViewController, BMKOfflineMapDelegate, UITextFieldDelegate, UITableViewDataSource, UITableViewDelegate  {
    
    @IBOutlet weak var cityIDLabel: UILabel!
    @IBOutlet weak var cityNameField: UITextField!
    @IBOutlet weak var cityListTableView: UITableView!
    @IBOutlet weak var downloadTableView: UITableView!
    
    /// 离线地图
    var offlineMap: BMKOfflineMap!
    /// 热门城市
    var hotCityData: [BMKOLSearchRecord]!
    /// 全国支持离线地图的城市
    var offlineCityData: [BMKOLSearchRecord]!
    /// 本地下载的离线地图
    var localDownloadMapInfo: [BMKOLUpdateElement]!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 初始化离线地图服务
        offlineMap = BMKOfflineMap()
        // 获取热门城市
        hotCityData = offlineMap.getHotCityList() as! [BMKOLSearchRecord]
        // 获取支持离线下载城市列表
        offlineCityData = offlineMap.getOfflineCityList() as! [BMKOLSearchRecord]
        // 获取离线地图信息
        if let info = offlineMap.getAllUpdateInfo() {
            localDownloadMapInfo = info as! [BMKOLUpdateElement]
        } else {
            localDownloadMapInfo = Array(repeating: BMKOLUpdateElement(), count: 0)
        }
        cityListTableView.isHidden = false
        downloadTableView.isHidden = true
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        offlineMap.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        offlineMap.delegate = nil
    }

    // MARK: - IBAction
    
    @IBAction func onClickSearch(_ sender: UIButton) {
        hiddenKeyBoard()
        // 根据城市名获取城市信息，得到城市 ID
        var city = offlineMap.searchCity(cityNameField.text)
        if (city?.count)! > 0 {
            let oneCity = city?[0] as! BMKOLSearchRecord
            cityIDLabel.text = "\(oneCity.cityID)"
        }
    }
    
    // 开始下载离线包
    @IBAction func onClickStart(_ sender: UIButton) {
        hiddenKeyBoard()
        let cityId = Int32(cityIDLabel.text!)
        if cityId != nil {
            offlineMap.start(cityId!)
        }
    }
    // 停止下载离线包
    @IBAction func onClickStop(_ sender: UIButton) {
        hiddenKeyBoard()
        let cityId = Int32(cityIDLabel.text!)
        if cityId != nil {
            offlineMap.pause(cityId!)
        }
    }
    // 删除本地离线包
    @IBAction func onClickDelete(_ sender: UIButton) {
        hiddenKeyBoard()
        let cityId = Int32(cityIDLabel.text!)
        if cityId != nil {
            if offlineMap.remove(cityId!) {
                reloadDownloadTableView()
            }
        }
    }
    
    // 城市列表/下载管理切换
    @IBAction func segmentChanged(_ sender: UISegmentedControl) {
        hiddenKeyBoard()
        cityListTableView.isHidden = sender.selectedSegmentIndex == 1
        downloadTableView.isHidden = sender.selectedSegmentIndex == 0
        if sender.selectedSegmentIndex == 0 {
            cityListTableView.reloadData()
        } else {
            reloadDownloadTableView()
        }
    }
    
    // MARK: - BMKOfflineMapDelegate

    /**
    *返回通知结果
    *@param type 事件类型： TYPE_OFFLINE_UPDATE,TYPE_OFFLINE_ZIPCNT,TYPE_OFFLINE_UNZIP, TYPE_OFFLINE_ERRZIP, TYPE_VER_UPDATE, TYPE_OFFLINE_UNZIPFINISH, TYPE_OFFLINE_ADD
    *@param state 事件状态，当type为TYPE_OFFLINE_UPDATE时，表示正在下载或更新城市id为state的离线包，当type为TYPE_OFFLINE_ZIPCNT时，表示检测到state个离线压缩包，当type为TYPE_OFFLINE_ADD时，表示新安装的离线地图数目，当type为TYPE_OFFLINE_UNZIP时，表示正在解压第state个离线包，当type为TYPE_OFFLINE_ERRZIP时，表示有state个错误包，当type为TYPE_VER_UPDATE时，表示id为state的城市离线包有更新，当type为TYPE_OFFLINE_UNZIPFINISH时，表示扫瞄完成，成功导入state个离线包
    */
    func onGetOfflineMapState(_ type: Int32, withState state: Int32) {
        
        if type == Int32(TYPE_OFFLINE_UPDATE) {
            //id为state的城市正在下载或更新，start后会毁掉此类型
            if let updateInfo = offlineMap .getUpdateInfo(state) {
                print("城市名：\(updateInfo.cityName)，下载比例：\(updateInfo.ratio)")
                if downloadTableView.isHidden == false {
                    reloadDownloadTableView()
                }
            }
            return
        }
        
        
        if type == Int32(TYPE_OFFLINE_NEWVER) {
            //id为state的state城市有新版本,可调用update接口进行更新
            if let updateInfo = offlineMap .getUpdateInfo(state) {
                print("是否有更新\(updateInfo.update)")
            }
            return
        }
        
        //正在解压第state个离线包，导入时会回调此类型
        if type == Int32(TYPE_OFFLINE_UNZIP) {
            return
        }
        
        //检测到state个离线包，开始导入时会回调此类型
        if type == Int32(TYPE_OFFLINE_ZIPCNT) {
            print("检测到%d个离线包",state);
            if state == 0 {
                showImportMessage(state)
            }
        }
        
        //有state个错误包，导入完成后会回调此类型
        if type == Int32(TYPE_OFFLINE_ERRZIP) {
            print("有%d个离线包导入错误",state);
        }
        
        //导入成功state个离线包，导入成功后会回调此类型
        if type == Int32(TYPE_OFFLINE_UNZIPFINISH) {
            print("成功导入%d个离线包",state);
            showImportMessage(state)
        }
    }
    
    // MARK: - UITableViewDataSource, UITableViewDelegate
    
    func numberOfSections(in tableView: UITableView) -> Int {
        if tableView == cityListTableView {
            return 2
        }else {
            return 1
        }
    }
    
    func tableView(_ tableView: UITableView, titleForHeaderInSection section: Int) -> String? {
        if tableView == cityListTableView {
            // 定义每个分组的标题
            var title = ""
            switch section {
            case 0:
                title = "热门城市"
            case 1:
                title = "全国"
            default:
                title = ""
            }
            return title
        }
        return nil
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        if tableView == cityListTableView {
            switch section {
            case 0:
                return hotCityData.count
            case 1:
                return offlineCityData.count
            default:
                return 0
            }
        }else if tableView == downloadTableView {
            return localDownloadMapInfo.count
        }
        return 0
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cellIdentifier = "OfflineMapCityCell"
        var cell = tableView.dequeueReusableCell(withIdentifier: cellIdentifier) 
        if cell == nil {
            cell = UITableViewCell(style: .default, reuseIdentifier: cellIdentifier)
        }
        if tableView == cityListTableView {
            // 热门城市列表
            if indexPath.section == 0 {
                let item = hotCityData[indexPath.row] as BMKOLSearchRecord
                cell?.textLabel?.text = "\(item.cityName)(\(item.cityID))"
                // 转换包大小
                let packSize = getDataSizeString(Int32(item.size))
                let sizeLabel = UILabel(frame: CGRect(x: 250, y: 0, width: 60, height: 40))
                sizeLabel.autoresizingMask = UIViewAutoresizing.flexibleLeftMargin
                sizeLabel.text = packSize
                sizeLabel.backgroundColor = UIColor.clear
                cell?.accessoryView = sizeLabel
            }
                // 支持离线下载城市列表
            else if indexPath.section == 1 {
                let item = offlineCityData[indexPath.row] as BMKOLSearchRecord
                cell?.textLabel?.text = "\(item.cityName)(\(item.cityID))"
                // 转换包大小
                let packSize = getDataSizeString(Int32(item.size))
                let sizeLabel = UILabel(frame: CGRect(x: 250, y: 0, width: 60, height: 40))
                sizeLabel.autoresizingMask = UIViewAutoresizing.flexibleLeftMargin
                sizeLabel.text = packSize
                sizeLabel.backgroundColor = UIColor.clear
                cell?.accessoryView = sizeLabel
            }
        }
        else {
            if localDownloadMapInfo != nil  {
                let item = localDownloadMapInfo[indexPath.row] as BMKOLUpdateElement
                // 单元格右侧文字
                let packSize = getDataSizeString(Int32(item.size))
                let sizeLabel = UILabel(frame: CGRect(x: 250, y: 0, width: 110, height: 40))
                sizeLabel.backgroundColor = UIColor.clear
                sizeLabel.autoresizingMask = UIViewAutoresizing.flexibleLeftMargin
                
                if item.ratio == 100{
                    sizeLabel.text = packSize
                } else {
                    let currentSize = getDataSizeString(Int32(item.size / 100 * Int64(item.ratio)))
                    sizeLabel.text = currentSize + "/" + packSize
                }
                if item.update {
                    sizeLabel.text = "可更新"
                    sizeLabel.textColor = UIColor.red
                }
                
                cell?.textLabel?.text = "\(item.cityName)"
                cell?.accessoryView = sizeLabel
            }else {
                cell?.textLabel?.text = ""
            }
        }
        return cell!
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        hiddenKeyBoard()
        
        if tableView == downloadTableView {
            let item = localDownloadMapInfo[indexPath.row] as BMKOLUpdateElement
            cityIDLabel.text = "\(item.cityID)"
        }else {
            // 获得当前选中的城市信息
            if indexPath.section == 0 {
                let item = hotCityData[indexPath.row] as BMKOLSearchRecord
                print("热门城市：\(item.cityName)(\(item.cityID))--包大小：\(getDataSizeString(Int32(item.size)))")
                cityIDLabel.text = "\(item.cityID)"
            }else if indexPath.section == 1 {
                let item = offlineCityData[indexPath.row] as BMKOLSearchRecord
                // 显示子单元格
                if item.childCities != nil && item.childCities.count > 0 {
                    var flag = true
                    for childitem in item.childCities as! [BMKOLSearchRecord] {
                        let tempString = "\(childitem.cityName)(\(childitem.cityID))"
                        // 转换包大小
                        let tempPackSize = getDataSizeString(Int32(childitem.size))
                        print("支持离线包城市：\(tempString)--包大小：\(tempPackSize)")
                        if flag {
                            cityIDLabel.text = "\(item.cityID)"
                            flag = false
                        }
                    }
                }
            }
        }
    }
    
    // MARK: - UITextFieldDelegate
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        hiddenKeyBoard()
        return true
    }
    
    // MARK: -
    
    //包大小转换工具（将包大小转换成合适的单位）
    func getDataSizeString(_ nSize: Int32) -> String {
        var string = ""
        if nSize < 1024 {
            string = "\(nSize)B"
        }else if nSize < 1048576 {
            string = "\(nSize/1024)K"
        }else if nSize < 1073741824 {
            if nSize % 1048576 == 0 {
                string = "\(nSize/1048576)M"
            }else {
                var decimal = 0 // 小数
                var decimalString = ""
                decimal =  Int(nSize) % 1048576
                decimal /= 1024
                
                if decimal < 10 {
                    decimalString = "0"
                }else if decimal >= 10 && decimal < 100 {
                    let i = decimal / 10
                    if i >= 5 {
                        decimalString = "1"
                    }else {
                        decimalString = "0"
                    }
                }
                else if decimal >= 100 && decimal < 1024 {
                    let i = decimal / 100
                    if i >= 5 {
                        decimal = i + 1
                        
                        if decimal >= 10 {
                            decimal = 9
                        }
                        decimalString = "\(decimal)"
                    }
                    else {
                        decimalString = "\(i)"
                    }
                }
                if decimalString == "" {
                    string = "\(nSize / 1048576)Mss"
                }else {
                    string = "\(nSize / 1048576).\(decimalString)M"
                }
            }
        }
            // > 1G
        else {
            string = "\(nSize / 1073741824)Gb"
        }
        
        return string
    }
    
    //隐藏键盘
    func hiddenKeyBoard() {
        cityNameField.resignFirstResponder()
    }
    
    // 导入提示框
    func showImportMessage(_ count: Int32) {
        let alertView = UIAlertController(title: "导入离线地图", message: "成功导入离线地图包个数:\(count)", preferredStyle: .alert)
        let okaction = UIAlertAction(title: "确定", style: UIAlertActionStyle.default, handler: nil)
        alertView.addAction(okaction)
        self.present(alertView, animated: true, completion: nil)
    }
    
    func reloadDownloadTableView() {
        // 获取各城市离线地图更新信息
        if let info = offlineMap.getAllUpdateInfo() {
            localDownloadMapInfo = info as! [BMKOLUpdateElement]
        }else {
            localDownloadMapInfo = Array(repeating: BMKOLUpdateElement(), count: 0)
        }
        downloadTableView.reloadData()
    }
}
