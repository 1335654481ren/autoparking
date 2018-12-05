//
//  StopView.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/17.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation
import UIKit
import Alamofire
import SwiftyJSON

class StopView: NNBaseViewController{
    //用户密码输入框
    var txtUser:UITextField!
    var txtPwd:UITextField!
    var button1:UIButton!

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
        txtPwd.text = levelArr?[station_num] as? String
        txtPwd.layer.cornerRadius = 9
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
        txtPwd.leftView!.addSubview(imgPwd)
        vLogin.addSubview(txtPwd)
        ///longin
        button1 = UIButton(frame: CGRect(x: mainSize.width/2 - 50, y: 430, width: 100, height: 40))
        button1.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
        button1.setTitle("结束", for: .normal)
        
        button1.setTitleColor(#colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1), for: .normal)
        button1.setTitleColor(#colorLiteral(red: 0.8039215803, green: 0.8039215803, blue: 0.8039215803, alpha: 1), for: .highlighted)
        button1.layer.cornerRadius = 9
        button1.layer.masksToBounds = true
        button1.titleLabel?.font = UIFont .systemFont(ofSize: 20.0)
        self.view.addSubview(button1)
        
        button1.addTarget(self, action: #selector(StopView.onClickStop), for: UIControlEvents.touchUpInside)
    }
    
    func onClickStop(){
        //dismiss(animated: true, completion: nil)Stop
        if(process == false ){
            self.navigationController?.popViewController(animated: true)
            return
        }else{
            var root = JSON()
            root["device_id"].string  = device_id
            let jsonstr = root.rawString()
            
            let manager = Alamofire.SessionManager.default
            manager.session.configuration.timeoutIntervalForRequest = 3
            
            let url = URL(string: appUrl + "api/v1/app/abortprocess")!
            var request = URLRequest(url: url)
            request.httpMethod = HTTPMethod.post.rawValue
            request.setValue("keep-alive", forHTTPHeaderField: "Connection")
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
            request.setValue("'Bearer \(String(describing: tokenStr))", forHTTPHeaderField: "Authorization")
            request.httpBody = jsonstr?.data(using: .utf8, allowLossyConversion: false)
            
            manager.request(request).response { response in
                    
                    if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                        
                        let jsondata = JSON(response.data ?? data)
                        print("code :\(jsondata["code"].int32Value)")
                        if(jsondata["code"].int32Value == 1){
                            //self.mesage_show(msg: "订单已结束")
                            process = false
                            gsocket.disconnect()
                            self.navigationController?.popViewController(animated: true)
                        }else{
                            self.mesage_show(msg: jsondata["message"].stringValue)
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
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
}
