//
//  loginViewController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/10/26.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation

import UIKit

import Alamofire
import SwiftyJSON
import WebSocket

class LoginViewController: UIViewController, UITextFieldDelegate {
    //用户密码输入框
    var txtUser:UITextField!
    var txtPwd:UITextField!
    
    //左手离脑袋的距离
    var offsetLeftHand:CGFloat = 60
    
    //左手图片,右手图片(遮眼睛的)
    var imgLeftHand:UIImageView!
    var imgRightHand:UIImageView!
    
    //左手图片,右手图片(圆形的)
    var imgLeftHandGone:UIImageView!
    var imgRightHandGone:UIImageView!
    
    //登录框状态
    var showType:LoginShowType = LoginShowType.NONE
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.view.backgroundColor = #colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1)
        //获取屏幕尺寸
        let mainSize = UIScreen.main.bounds.size
        //猫头鹰头部
        let imgLogin =  UIImageView(frame:CGRect(x:mainSize.width/2-211/2, y:100, width:211, height:109))
        imgLogin.image = UIImage(named:"owl-login")
        imgLogin.layer.masksToBounds = true
        self.view.addSubview(imgLogin)
        
        //猫头鹰左手(遮眼睛的)
        let rectLeftHand = CGRect(x:61 - offsetLeftHand, y:90, width:40, height:65)
        imgLeftHand = UIImageView(frame:rectLeftHand)
        imgLeftHand.image = UIImage(named:"owl-login-arm-left")
        imgLogin.addSubview(imgLeftHand)
        
        //猫头鹰右手(遮眼睛的)
        let rectRightHand = CGRect(x:imgLogin.frame.size.width / 2 + 60, y:90, width:40, height:65)
        imgRightHand = UIImageView(frame:rectRightHand)
        imgRightHand.image = UIImage(named:"owl-login-arm-right")
        imgLogin.addSubview(imgRightHand)
        
        //登录框背景
        let vLogin =  UIView(frame:CGRect(x:15, y:200, width:mainSize.width - 30, height:160))
        vLogin.layer.cornerRadius = 15
        vLogin.layer.borderWidth = 0.5
        vLogin.layer.borderColor = UIColor.lightGray.cgColor
        vLogin.backgroundColor = #colorLiteral(red: 1, green: 1, blue: 1, alpha: 1)
        self.view.addSubview(vLogin)
        
        //猫头鹰左手(圆形的)
        let rectLeftHandGone = CGRect(x:mainSize.width / 2 - 100,
                                      y:vLogin.frame.origin.y - 22, width:40, height:40)
        imgLeftHandGone = UIImageView(frame:rectLeftHandGone)
        imgLeftHandGone.image = UIImage(named:"icon_hand")
        self.view.addSubview(imgLeftHandGone)
        
        //猫头鹰右手(圆形的)
        let rectRightHandGone = CGRect(x:mainSize.width / 2 + 62,
                                       y:vLogin.frame.origin.y - 22, width:40, height:40)
        imgRightHandGone = UIImageView(frame:rectRightHandGone)
        imgRightHandGone.image = UIImage(named:"icon_hand")
        self.view.addSubview(imgRightHandGone)
        
        //用户名输入框
        txtUser = UITextField(frame:CGRect(x:30, y:30, width:vLogin.frame.size.width - 60, height:44))
        txtUser.delegate = self
        txtUser.placeholder = "输入电话号码"
        txtUser.text = "13366470670"
        txtUser.layer.cornerRadius = 9
        txtUser.layer.borderColor = UIColor.lightGray.cgColor
        txtUser.layer.borderWidth = 0.5
        txtUser.leftView = UIView(frame:CGRect(x:0, y:0, width:44, height:44))
        txtUser.leftViewMode = UITextFieldViewMode.always
        
        //用户名输入框左侧图标
        let imgUser =  UIImageView(frame:CGRect(x:11, y:11, width:22, height:22))
        imgUser.image = UIImage(named:"iconfont-user")
        txtUser.leftView!.addSubview(imgUser)
        vLogin.addSubview(txtUser)
        
        //密码输入框
        txtPwd = UITextField(frame:CGRect(x:30, y:90, width:vLogin.frame.size.width - 60, height:44))
        txtPwd.delegate = self
        txtPwd.placeholder = "输入密码"
        txtPwd.text = "12345678ren"
        txtPwd.layer.cornerRadius = 9
        txtPwd.layer.borderColor = UIColor.lightGray.cgColor
        txtPwd.layer.borderWidth = 0.5
        txtPwd.isSecureTextEntry = true
        txtPwd.leftView = UIView(frame:CGRect(x:0, y:0, width:44, height:44))
        txtPwd.leftViewMode = UITextFieldViewMode.always
        
        //密码输入框左侧图标
        let imgPwd =  UIImageView(frame:CGRect(x:11, y:11, width:22, height:22))
        imgPwd.image = UIImage(named:"iconfont-password")
        txtPwd.leftView!.addSubview(imgPwd)
        vLogin.addSubview(txtPwd)
        ///longin
        let button1 = UIButton(frame: CGRect(x: 70, y: 370, width: 100, height: 40))
        button1.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
        button1.setTitle("登录", for: .normal)
        button1.setTitleColor(#colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1), for: .normal)
        button1.setTitleColor(#colorLiteral(red: 0.8039215803, green: 0.8039215803, blue: 0.8039215803, alpha: 1), for: .highlighted)
        button1.layer.cornerRadius = 9
        button1.layer.masksToBounds = true
        button1.titleLabel?.font = UIFont .systemFont(ofSize: 20.0)
        self.view.addSubview(button1)
        
        let button2 = UIButton(frame: CGRect(x: 230, y: 370, width: 100, height: 40))
        button2.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
        button2.setTitle("注册", for: .normal)
        button2.setTitleColor(#colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1), for: .normal)
        button2.setTitleColor(#colorLiteral(red: 0.8039215803, green: 0.8039215803, blue: 0.8039215803, alpha: 1), for: .highlighted)
        button2.layer.cornerRadius = 9
        button2.layer.masksToBounds = true
        button2.titleLabel?.font = UIFont .systemFont(ofSize: 20.0)
        self.view.addSubview(button2)
        button1.addTarget(self, action: #selector(LoginViewController.onClickLogin), for: UIControlEvents.touchUpInside)
        button2.addTarget(self, action: #selector(LoginViewController.onClickRegister), for: UIControlEvents.touchUpInside)
 
    }
    
    func onClickLogin(){

        var data = JSON()
        data["phone_num"].string = txtUser.text
        data["password"].string = txtPwd.text
        let params = data.rawString()
        print("onClickLogin")
        let url = URL(string: appUrl + "api/v1/app/login")!
        var request = URLRequest(url: url)
        request.httpMethod = HTTPMethod.post.rawValue
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.setValue("Basic dXNlcjoxMjM0NQ==", forHTTPHeaderField: "Authorization")
        request.httpBody = params?.data(using: .utf8, allowLossyConversion: false)
        
        Alamofire.request(request).response { response in

                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("Login code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let data = jsondata["data"]
                        tokenStr = data["token"].stringValue
                        //print("token :\(String(describing: tokenStr))")
                        logon = true
                        //请求未完成的订单
                        self.request_have_process()
                        self.view.window?.rootViewController = NNTabBarController()
                    }else{
                        let alertVC = UIAlertController(title: "", message: jsondata["message"].stringValue, preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                        //跳转新页面
                        //self.present(NNSnapKitController(), animated: true, completion: nil)
                        //self.view.window?.rootViewController = NNTabBarController()
                    }
                }
        }
    }
    
    func onClickRegister(){

        var data = JSON()
        data["phone_num"].string = txtUser.text
        data["password"].string = txtPwd.text
        let params = data.rawString()
        print("onClickRegister")
        let url = URL(string: appUrl + "api/v1/app/register")!
        var request = URLRequest(url: url)
        request.httpMethod = HTTPMethod.post.rawValue
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.setValue("Basic dXNlcjoxMjM0NQ==", forHTTPHeaderField: "Authorization")
        request.httpBody = params?.data(using: .utf8, allowLossyConversion: false)

        Alamofire.request(request).response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print("code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let alertVC = UIAlertController(title: "", message: "注册成功", preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                    }else{
                        let alertVC = UIAlertController(title: "", message: jsondata["message"].stringValue, preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                    }
                }
        }
    }
    func request_have_process(){
        
        let headers: HTTPHeaders = [
            "Connection" : "keep-alive",
            "Authorization" : "'Bearer \(String(describing: tokenStr))",
            "Content-Type": "application/json"
        ]
        Alamofire.request(appUrl + "api/v1/app/process",method:.get,headers:headers)
            .response { response in
                
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    let jsondata = JSON(response.data ?? data)
                    print(jsondata)
                    print("request_have_process code :\(jsondata["code"].int32Value)")
                    if(jsondata["code"].int32Value == 1){
                        let alertVC = UIAlertController(title: "", message: "你有一个为完成的订单,请确认", preferredStyle: .alert)
                        let alertAction = UIAlertAction(title: "Ok", style: .cancel, handler: nil)
                        alertVC.addAction(alertAction)
                        self.present(alertVC, animated: true, completion: nil)
                        var car = jsondata["data"]
                        car_type = car["car_type"].stringValue
                        license = car["license"].stringValue
                        device_id = car["device_id"].stringValue
                        route = car["route"].arrayObject as? Array<Int>
                        let flowstage = car["flow_stage"].int32Value
                        print("flowstage:\(flowstage)")
                        if(flowstage == 3 ){
                            print("..............")
                            gflowstage = 3
                            process = true
                        }else if(flowstage == 2){
                            process = true
                            self.mesage_show(msg:"the car is on the way")
                            if( gsocket.isConnected == false){
                                print("web socket connect .......")
                                let token = tokenStr
                                gsocket.request.setValue(token, forHTTPHeaderField: "Authorization")
                                gsocket.connect()
                                NSLog("websocket connected!!!!!!")
                            }
                        }
                    }else{
                        //self.mesage_show(msg:jsondata["message"].stringValue)
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

    //输入框获取焦点开始编辑
    func textFieldDidBeginEditing(_ textField:UITextField)
    {
        //如果当前是用户名输入
        if textField.isEqual(txtUser){
            if (showType != LoginShowType.PASS)
            {
                showType = LoginShowType.USER
                return
            }
            showType = LoginShowType.USER
            
            //播放不遮眼动画
            UIView.animate(withDuration: 0.5, animations: { () -> Void in
                self.imgLeftHand.frame = CGRect(
                    x:self.imgLeftHand.frame.origin.x - self.offsetLeftHand,
                    y:self.imgLeftHand.frame.origin.y + 30,
                    width:self.imgLeftHand.frame.size.width, height:self.imgLeftHand.frame.size.height)
                self.imgRightHand.frame = CGRect(
                    x:self.imgRightHand.frame.origin.x + 48,
                    y:self.imgRightHand.frame.origin.y + 30,
                    width:self.imgRightHand.frame.size.width, height:self.imgRightHand.frame.size.height)
                self.imgLeftHandGone.frame = CGRect(
                    x:self.imgLeftHandGone.frame.origin.x - 70,
                    y:self.imgLeftHandGone.frame.origin.y, width:40, height:40)
                self.imgRightHandGone.frame = CGRect(
                    x:self.imgRightHandGone.frame.origin.x + 30,
                    y:self.imgRightHandGone.frame.origin.y, width:40, height:40)
            })
        }
            //如果当前是密码名输入
        else if textField.isEqual(txtPwd){
            if (showType == LoginShowType.PASS)
            {
                showType = LoginShowType.PASS
                return
            }
            showType = LoginShowType.PASS
            
            //播放遮眼动画
            UIView.animate(withDuration: 0.5, animations: { () -> Void in
                self.imgLeftHand.frame = CGRect(
                    x:self.imgLeftHand.frame.origin.x + self.offsetLeftHand,
                    y:self.imgLeftHand.frame.origin.y - 30,
                    width:self.imgLeftHand.frame.size.width, height:self.imgLeftHand.frame.size.height)
                self.imgRightHand.frame = CGRect(
                    x:self.imgRightHand.frame.origin.x - 48,
                    y:self.imgRightHand.frame.origin.y - 30,
                    width:self.imgRightHand.frame.size.width, height:self.imgRightHand.frame.size.height)
                self.imgLeftHandGone.frame = CGRect(
                    x:self.imgLeftHandGone.frame.origin.x + 70,
                    y:self.imgLeftHandGone.frame.origin.y,width: 0, height:0)
                self.imgRightHandGone.frame = CGRect(
                    x:self.imgRightHandGone.frame.origin.x - 30,
                    y:self.imgRightHandGone.frame.origin.y, width:0, height:0)
            })
        }
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
}

//登录框状态枚举
enum LoginShowType {
    case NONE
    case USER
    case PASS
}
