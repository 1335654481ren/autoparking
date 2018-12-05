//
//  AppDelegate.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.

import UIKit
import WebSocket
import CocoaMQTT
import SwiftyJSON

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate, BMKGeneralDelegate, BMKLocationAuthDelegate ,WebSocketDelegate{
    var _mapManager: BMKMapManager?
    var window: UIWindow?
    var mqtt: CocoaMQTT?
    let defaultHost = "tcp://ngrok.xiaomiqiu.cn"
    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplicationLaunchOptionsKey: Any]?) -> Bool {
        // MARK: - 设置跟控制器
        window = UIWindow(frame: UIScreen.main.bounds)
        window?.makeKeyAndVisible()
        
        // Override point for customization after application launch.
        // 初始化定位SDK
        BMKLocationAuth.sharedInstance().checkPermision(withKey: "tpbW8KvweBbO4X0B35yERcts9NfnnBaG", authDelegate: self)
        // 要使用百度地图，请先启动BaiduMapManager
        _mapManager = BMKMapManager()

        if BMKMapManager.setCoordinateTypeUsedInBaiduMapSDK(BMK_COORD_TYPE.COORDTYPE_BD09LL) {
            NSLog("经纬度类型设置成功");
        } else {
            NSLog("经纬度类型设置失败");
        }

        // 如果要关注网络及授权验证事件，请设定generalDelegate参数
        let ret = _mapManager?.start("tpbW8KvweBbO4X0B35yERcts9NfnnBaG", generalDelegate: self as BMKGeneralDelegate)
        if ret == false {
            NSLog("manager start failed!")
        }else{
            NSLog("manager start ok")
        }
        
        init_websoket()
        // 是能https ,忽略认证
        //let manager = HTTPSManager()
        //manager.setAlamofireHttps()
//        let manager = SessionManager.default
//        manager.delegate.sessionDidReceiveChallenge = {
//            session,challenge in
//            return    (URLSession.AuthChallengeDisposition.useCredential,URLCredential(trust:challenge.protectionSpace.serverTrust!))
//        }
        
        //window?.rootViewController = CallCarViewController()
        window?.rootViewController = LoginViewController()

        return true
    }
    
    func mqttSetting() {
        let clientID = "CocoaMQTT_test"
        mqtt = CocoaMQTT(clientID: clientID, host: defaultHost, port: 36508)
        mqtt!.username = ""
        mqtt!.password = ""
        mqtt!.willMessage = CocoaMQTTWill(topic: "/will", message: "dieout")
        mqtt!.keepAlive = 60
        mqtt!.delegate = self as? CocoaMQTTDelegate
    }
    
    func applicationWillResignActive(_ application: UIApplication) {
        // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
        // Use this method to pause ongoing tasks, disable timers, and invalidate graphics rendering callbacks. Games should use this method to pause the game.
        print("applicationWillResignActive")
    }

    func applicationDidEnterBackground(_ application: UIApplication) {
        // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
        // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
        print("applicationDidEnterBackground")
//        if( gsocket.isConnected == true && logon == true){
//            gsocket.disconnect()
//        }
    }

    func applicationWillEnterForeground(_ application: UIApplication) {
        // Called as part of the transition from the background to the active state; here you can undo many of the changes made on entering the background.
        print("applicationWillEnterForeground")
//        if( gsocket.isConnected == false && logon == true ){
//            gsocket.connect()
//        }
    }

    func applicationDidBecomeActive(_ application: UIApplication) {
        // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
        print("applicationDidBecomeActive")
    }

    func applicationWillTerminate(_ application: UIApplication) {
        // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
        print("applicationWillTerminate")
//        if( gsocket.isConnected ){
//            gsocket.disconnect()
//        }
    }
    //MARK: - BMKGeneralDelegate
    func onGetNetworkState(_ iError: Int32) {
        if (0 == iError) {
            NSLog("联网成功");
        }
        else{
            NSLog("联网失败，错误代码：Error\(iError)");
        }
    }
    
    func onGetPermissionState(_ iError: Int32) {
        if (0 == iError) {
            NSLog("授权成功");
        }
        else{
            NSLog("授权失败，错误代码：Error\(iError)");
        }
    }
    
    func init_websoket() {
        var request = URLRequest(url: URL(string: websocket_url)!)
        request.timeoutInterval = 5
        gsocket = WebSocket(request: request)
        gsocket.delegate = self
        //        socket.enableCompression = false //Compression Extensions (RFC 7692)
        //        //create a custom queue
        //        socket.callbackQueue = DispatchQueue(label: "com.vluxe.starscream.myapp")
        //        // Set enabled cipher suites to AES 256 and AES 128
        //        socket.enabledSSLCipherSuites = [TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384, TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256]
        //        let data = sll.cert //load your certificate from disk
        //        socket.security = SSLSecurity(certs: [SSLCert(data: data)], usePublicKeys: true)
        //        //socket.security = SSLSecurity() //uses the .cer files in your app's bundle
        //gsocket.request.addValue("Authorization", forHTTPHeaderField: tokenStr as! String)
        //gsocket.connect()
    }
    // MARK: Websocket Delegate Methods.
    internal func websocketDidConnect(socket: WebSocketClient) {
        print("websocket is connected")
    }
    
    internal func websocketDidDisconnect(socket: WebSocketClient, error: Error?) {
        if let e = error as? WSError {

            print("websocket is disconnected1: \(e.message)")
        } else if let e = error {
            print("websocket is disconnected2: \(e.localizedDescription)")
        } else {
            print("websocket disconnected3")
        }
    }
    
    internal func websocketDidReceiveMessage(socket: WebSocketClient, text: String) {
        let  recv = text.data(using: .utf8, allowLossyConversion: false)
        let jsondata = JSON(recv)
//        print("Received text:")
//        print(jsondata)
//        print("version : \(jsondata["version"].stringValue)")
//        print("type : \(jsondata["type"].intValue)")
        if(jsondata["type"].int32Value == 1){
            //print("local.....")
            NotificationCenter.default.post(name: NSNotification.Name(rawValue: "local"), object: self, userInfo: ["content":text])
        }else if(jsondata["type"].int32Value == 2){
            //print("status ......")
            NotificationCenter.default.post(name: NSNotification.Name(rawValue: "status"), object: self, userInfo: ["content":text])
        }else if(jsondata["type"].int32Value == 3){
            print("arrived ......")
            NotificationCenter.default.post(name: NSNotification.Name(rawValue: "arrive"), object: self, userInfo: ["content":text])
        }
    }
    
    internal func websocketDidReceiveData(socket: WebSocketClient, data: Data) {
        print("Received data: \(data.count)")
    }
    
}
