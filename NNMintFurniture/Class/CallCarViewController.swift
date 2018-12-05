//
//  CallCarViewController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/17.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation
import UIKit
import Alamofire
import SwiftyJSON

class CallCarViewController: NNBaseViewController, UITextFieldDelegate{
        //用户密码输入框
        var txtUser:UITextField!
        var txtPwd:UITextField!
        var button1:UIButton!
        var button2:UIButton!
    
        lazy var menu: Menu = {
            var me = Menu.initMenu(size: CGSize(width:UIScreen.main.bounds.size.width / 3,height:self.view.frame.size.height / 6))
            me.layer.cornerRadius = 5
            self.view.addSubview(me)
            return me
        }()
    
        override func viewDidLoad() {
            super.viewDidLoad()
            self.view.backgroundColor = #colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1)
            //获取屏幕尺寸
            let mainSize = UIScreen.main.bounds.size
            //猫头鹰头部
            let imgLogin =  UIImageView(frame:CGRect(x:mainSize.width/2 - 211/2, y:100, width:211, height:150))
            imgLogin.image = UIImage(named:"audi")
            imgLogin.layer.cornerRadius = 50
            imgLogin.layer.masksToBounds = true
            self.view.addSubview(imgLogin)
 
            //登录框背景
            let vLogin =  UIView(frame:CGRect(x:60, y:260, width:mainSize.width - 120, height:160))
            vLogin.layer.cornerRadius = 15
            vLogin.layer.borderWidth = 0.5
            vLogin.layer.borderColor = UIColor.lightGray.cgColor
            vLogin.backgroundColor = #colorLiteral(red: 1, green: 1, blue: 1, alpha: 1)
            self.view.addSubview(vLogin)
            
            //用户名输入框
            txtUser = UITextField(frame:CGRect(x:30, y:30, width:vLogin.frame.size.width - 60, height:44))
            txtUser.placeholder = license
            txtUser.text = license
            txtUser.inputView = UIView()
            txtUser.becomeFirstResponder()
            txtUser.layer.cornerRadius = 9
            txtUser.layer.borderColor = UIColor.lightGray.cgColor
            txtUser.layer.borderWidth = 0.5
            txtUser.leftView = UIView(frame:CGRect(x:0, y:0, width:44, height:44))
            txtUser.leftViewMode = UITextFieldViewMode.always
            //用户名输入框左侧图标
            let imgUser =  UIImageView(frame:CGRect(x:11, y:11, width:22, height:22))
            imgUser.image = UIImage(named:"car")
            txtUser.leftView!.addSubview(imgUser)
            vLogin.addSubview(txtUser)
            
            //密码输入框
            txtPwd = UITextField(frame:CGRect(x:30, y:90, width:vLogin.frame.size.width - 60, height:44))
            txtPwd.placeholder = "输入站点"
            txtPwd.layer.cornerRadius = 9
            txtPwd.delegate = self
            txtPwd.inputView = UIView()
            txtPwd.becomeFirstResponder()
            txtPwd.layer.borderColor = UIColor.lightGray.cgColor
            txtPwd.layer.borderWidth = 0.5
            //txtPwd.isSecureTextEntry = true
            txtPwd.leftView = UIView(frame:CGRect(x:0, y:0, width:44, height:44))
            txtPwd.leftViewMode = UITextFieldViewMode.always

            //密码输入框左侧图标
            let imgPwd =  UIButton(frame:CGRect(x:11, y:11, width:22, height:22))
            imgPwd.setImage(UIImage(named:"station"), for: .normal)
            imgPwd.addTarget(self, action: #selector(CallCarViewController.select_menu), for: UIControlEvents.touchUpInside)
            
            txtPwd.leftView!.addSubview(imgPwd)
            vLogin.addSubview(txtPwd)
            ///longin
            button1 = UIButton(frame: CGRect(x: mainSize.width/2 - 50, y: 430, width: 100, height: 40))
            button1.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
            if(process == false){
                button1.setTitle("叫车", for: .normal)
            }else{
                button1.setTitle("返回", for: .normal)
            }
            
            button1.setTitleColor(#colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1), for: .normal)
            button1.setTitleColor(#colorLiteral(red: 0.8039215803, green: 0.8039215803, blue: 0.8039215803, alpha: 1), for: .highlighted)
            button1.layer.cornerRadius = 9
            button1.layer.masksToBounds = true
            button1.titleLabel?.font = UIFont .systemFont(ofSize: 20.0)
            self.view.addSubview(button1)
            
            button1.addTarget(self, action: #selector(CallCarViewController.onClickCalling), for: UIControlEvents.touchUpInside)
            
            ///cancel
            button2 = UIButton(frame: CGRect(x: mainSize.width/2 - 50, y: 480, width: 100, height: 40))
            button2.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
            button2.setTitle("返回", for: .normal)

            
            button2.setTitleColor(#colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1), for: .normal)
            button2.setTitleColor(#colorLiteral(red: 0.8039215803, green: 0.8039215803, blue: 0.8039215803, alpha: 1), for: .highlighted)
            button2.layer.cornerRadius = 9
            button2.layer.masksToBounds = true
            button2.titleLabel?.font = UIFont .systemFont(ofSize: 20.0)
            self.view.addSubview(button2)
            
            button2.addTarget(self, action: #selector(CallCarViewController.onCancel), for: UIControlEvents.touchUpInside)
        }
    
        func onCancel(){
            self.navigationController?.popViewController(animated: true)
        }
    
        func onClickCalling(){
            
            if(process == true){
                 self.navigationController?.popViewController(animated: true)
                 return
            }else{
                var root = JSON()
                var station = JSON()
                station["station_num"].int = station_num + 1
                station["x"].float = 11.11
                station["y"].float = 22.22
                station["z"].float = 33.33
                root["station"] = station
                root["device_id"].string  = device_id
                root["command"].int32 = 1
                let jsonstr = root.rawString()
                
                let manager = Alamofire.SessionManager.default
                manager.session.configuration.timeoutIntervalForRequest = 3
                
                let url = URL(string: appUrl + "api/v1/app/callcar")!
                var request = URLRequest(url: url)
                request.httpMethod = HTTPMethod.post.rawValue
                request.setValue("keep-alive", forHTTPHeaderField: "Connection")
                request.setValue("application/json", forHTTPHeaderField: "Content-Type")
                request.setValue("'Bearer \(String(describing: tokenStr))", forHTTPHeaderField: "Authorization")
                request.httpBody = jsonstr?.data(using: .utf8, allowLossyConversion: false)
                
                manager.request(request).response { response in
                        
                        if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                            
                            let jsondata = JSON(response.data ?? data)
                            print(jsondata)
                            print("code :\(jsondata["code"].int32Value)")
                            if(jsondata["code"].int32Value == 1){
                                var car = jsondata["data"]
                                route = car["route"].arrayObject as? Array<Int>
                                process = true
                                //self.mesage_show(msg: "The car is arriving!")
                                if( gsocket.isConnected == false){
                                        print("web socket connect .......")
                                        let token = tokenStr
                                        gsocket.request.setValue(token, forHTTPHeaderField: "Authorization")
                                        gsocket.connect()
                                        NSLog("websocket connected!!!!!!")
                                }
                                self.navigationController?.popViewController(animated: true)
                            }else{
                                self.mesage_show(msg: jsondata["message"].stringValue)
                                //self.navigationController?.popViewController(animated: true)
                            }
                        }
                }
            }
        }
    
        func mesage_show(msg:String){
            let alertVC = UIAlertController(title: "", message: msg, preferredStyle: .alert)
            let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
            alertVC.addAction(alertAction)
            self.present(alertVC, animated: true, completion: nil)
        }
    
        func select_menu(){

            let point = CGPoint(x:120,y: 400)
            
            let dictValues = [String](stationNameArry.values)
            //print(stationNameArry)
            self.menu.popupMenu(orginPoint:point, arr: dictValues)
            
            self.menu.didSelectIndex = { [unowned self] (index:Int) in
                station_num = index
                print( "选中--  \(index) -行 -- \(String(describing: dictValues[index]))")
                self.txtPwd.text = dictValues[index]
                self.menu.packUpMenu()
            }
        }
    
        //输入框获取焦点开始编辑
        func textFieldDidBeginEditing(_ textField:UITextField)
        {
            print("request map list")
            select_menu()
        }
    
        override func didReceiveMemoryWarning() {
            super.didReceiveMemoryWarning()
        }
}
