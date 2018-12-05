//
//  NNDownloadMapController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/23.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation
import UIKit
import Alamofire
import SwiftyJSON

class NNDownloadMapController: NNBaseViewController, UITextFieldDelegate {
    var button:UIButton!
    var declabel:UILabel!
    var map_id_text:UITextField!
    
    lazy var menu: Menu = {
        var sized = CGSize(width:UIScreen.main.bounds.size.width - 40,height:90 )
        var me = Menu.initMenu(size: sized)
        me.layer.cornerRadius = 5
        self.view.addSubview(me)
        return me
    }()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        title = "DownLoadMap"
        view.backgroundColor = UIColor.gray
        navigationController?.navigationBar.isTranslucent = false
        
        
        let label = UILabel()
        label.frame = CGRect(x:self.view.bounds.size.width/2 - 50,y:80,width:100,height:50)
        label.text = "MapID"
        label.font = UIFont.systemFont(ofSize:20)
        label.textAlignment = NSTextAlignment.center
        
        self.view.addSubview(label)
        
        map_id_text = UITextField()
        map_id_text.placeholder = "请输入MapID"
        //map_id_text.text = "c066ffec-3ec7-4b68-8bf1-593711ba6dd4"
        map_id_text.frame = CGRect(x:10,y:140,width:self.view.bounds.size.width - 20,height:50)
        map_id_text.layer.borderWidth = 0.5
        map_id_text.layer.cornerRadius = 5
        map_id_text.delegate = self
        map_id_text.adjustsFontSizeToFitWidth=true
        map_id_text.textAlignment = .center
        self.view.addSubview(map_id_text)

        button = UIButton()
        button.frame = CGRect(x:self.view.bounds.size.width/2 - 40,y:200,width:80,height:50)
        button.setTitle("开始下载", for: .normal)
        button.layer.cornerRadius = 5
        button.addTarget(self, action: #selector(self.onclice), for: .touchUpInside)
        self.view.addSubview(button)

        declabel = UILabel()
        declabel.frame = CGRect(x:10,y:260,width:self.view.bounds.size.width - 20,height:100)
        declabel.text = "地图说明"
        declabel.numberOfLines = 0
        declabel.font = UIFont.systemFont(ofSize:20)
        declabel.textAlignment = NSTextAlignment.center
        
        self.view.addSubview(declabel)
    }
    
    func mesage_show(msg:String) -> Bool{
        let alertVC = UIAlertController(title: "", message: msg, preferredStyle: .alert)
        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
        alertVC.addAction(alertAction)
        self.present(alertVC, animated: true, completion: nil)
        return true
    }
    
    func onclice(){
        print("开始下载。。。。。。")
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        if( (self.map_id_text.text?.isEmpty)!){
            mesage_show(msg: "请输入MapID")
            return
        }
        let url = appUrl + "api/v1/app/map/" + self.map_id_text.text!
        
        Alamofire.request(url,method:.get,headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    if( jsondata["code"].int32Value == 1 ){
                    
                        var data = jsondata["data"]
                        let base64 = data["map"].stringValue
                        let xml = self.base64Decoding(encodedString: base64)
                        //print("xml:\(xml)")

                        let filepath:String! = NSHomeDirectory() + "/Documents/zgc.xodr"
                        //print("path :\(filepath)")
                        
                        let url = URL(fileURLWithPath: filepath)
                        do{
                            _ = try xml.write(to: url, atomically: true, encoding: String.Encoding.utf8)
                        }catch{
                            self.mesage_show(msg:"download map error!")
                        }
                        
                        self.declabel.text = data["description"].stringValue
                        //var flag = self.mesage_show(msg: "重启软件生效新地图")
                        self.showActionSheet()
                    }else{
                        print("request cars :\(jsondata["message"].stringValue)")
                        self.mesage_show(msg: jsondata["message"].stringValue)
                    }
                }
        }
    }
    func showActionSheet()  {
        let alert = UIAlertController(title:"",message:"地图更新完成重启生效，是否重启？",preferredStyle:UIAlertControllerStyle.actionSheet)
        
        let yes = UIAlertAction(title:"立刻重启",style:UIAlertActionStyle.default,handler:{(alerts:UIAlertAction) -> Void in
            //暂时理解为回调
            print("action ok")
            exit(0)
        })

        let no  = UIAlertAction(title:"稍后重启",style:UIAlertActionStyle.destructive,handler:{(alerts:UIAlertAction) -> Void in
            print("action no")})
        
//        let unKnown = UIAlertAction(title:"undo" ,style:UIAlertActionStyle.cancel,handler:{(alerts:UIAlertAction) -> Void in
//            print("不知道什么操作")
//        })
        
        
        alert.addAction(yes)
        alert.addAction(no)
        //alert.addAction(unKnown)
        
        self.present(alert,animated: true,completion: nil)
        
    }
    
    func request_map_list(){
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        Alamofire.request(appUrl + "api/v1/app/map_list",method:.get,headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data station: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print(jsondata)
                    print("request_map_list code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        var map_description = [String]()
                        var map_list = [Int : String]()
                        let stations = jsondata["data"]["records"].arrayValue
                        var cnt:Int = 0
                        for item in stations {
                            map_description.append(item["description"].stringValue)
                            map_list[cnt] = item["map_id"].stringValue
                            cnt = cnt + 1
                        }
                        let point = CGPoint(x:20,y: 190)
                        print(map_description)
                        self.menu.popupMenu(orginPoint:point, arr: map_description )
                        self.menu.layer.cornerRadius = 5
                        self.menu.didSelectIndex = { [unowned self] (index:Int) in
                            station_num = index
                            print( "选中--  \(index) -行 -- \(String(describing: map_description[index]))")
                            self.map_id_text.text = map_list[index]
                            self.menu.packUpMenu()
                        }
                    }else{
                        self.mesage_show(msg:jsondata["message"].stringValue)
                    }
                }
        }
    }
    
    //输入框获取焦点开始编辑
    func textFieldDidBeginEditing(_ textField:UITextField)
    {
        print("request map list")
        request_map_list()
    }

    func base64Decoding(encodedString:String)->String
    {
        let decodedData = NSData(base64Encoded: encodedString, options: NSData.Base64DecodingOptions.init(rawValue: 0))
        let decodedString = NSString(data: decodedData! as Data, encoding: String.Encoding.utf8.rawValue)! as String
        return decodedString
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
}
