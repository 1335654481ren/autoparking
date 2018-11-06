//
//  NetInterface.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/10/11.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import SwiftHTTP
import Alamofire
import WebSocket
import SwiftyJSON

class Mysocket: WebSocketDelegate {
    var socket: WebSocket!
    
    func init_websoket() {
        var request = URLRequest(url: URL(string: "renx")!)
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket.delegate = self
        socket.connect()
    }
    // MARK: Websocket Delegate Methods.
    internal func websocketDidConnect(socket: WebSocketClient) {
        print("websocket is connected")
    }
    
    internal func websocketDidDisconnect(socket: WebSocketClient, error: Error?) {
        if let e = error as? WSError {
            print("websocket is disconnected: \(e.message)")
        } else if let e = error {
            print("websocket is disconnected: \(e.localizedDescription)")
        } else {
            print("websocket disconnected")
        }
    }
    
    internal func websocketDidReceiveMessage(socket: WebSocketClient, text: String) {
        print("Received text: \(text)")
    }
    
    internal func websocketDidReceiveData(socket: WebSocketClient, data: Data) {
        print("Received data: \(data.count)")
    }
    
    // MARK: Write Text Action
    func websoket_send(data:String) {
        socket.write(string:data)
    }
    
    // MARK: Disconnect Action
    func disconnect() {
        if socket.isConnected {
            socket.disconnect()
        } else {
            socket.connect()
        }
    }
    
}

func interface_test(){
    var mWebSocket: Mysocket?
    
    mWebSocket = Mysocket.init()
    mWebSocket?.init_websoket()
    
    let headers: HTTPHeaders = [
        "Host" : "ihome.ngrok.xiaomiqiu.cn",
        "Connection" : "keep-alive",
        "Authorization" : "Basic dXNlcjoxMjM0NQ==",
        "Accept": "application/json"
    ]
    
    Alamofire.request("http://ihome.ngrok.xiaomiqiu.cn/asp/login.asp",method:.get, parameters: ["foo": "bar"],headers:headers)
        .response { response in
            print("Request: \(response.request)")
            print("Response: \(response.response)")
            print("Error: \(response.error)")
            
            if let data = response.data, let utf8Text = String(data: data, encoding: .utf8) {
                print("Data: \(utf8Text)")
            }
    }
    
}
/// 请求响应状态
///
/// - success: 响应成功
/// - unusual: 响应异常
/// - failure: 请求错误
enum ResponseStatus: Int {
    case success  = 0
    case unusual  = 1
    case failure  = 3
}

/// 网络请求回调闭包 status:响应状态 result:JSON tipString:提示给用户的信息
typealias NetworkFinished = (_ status: ResponseStatus, _ result: JSON?, _ tipString: String?) -> ()

class NetworkTools: NSObject {
    
    /// 网络工具类单例
    static let shared = NetworkTools()
    
    var rootDomain: String {
        return "http://ihome.ngrok.xiaomiqiu.cn"
    }
    
}

// MARK: - 基础请求方法
extension NetworkTools {
    //    in view: UIView = UIApplication.shared.keyWindow!,title: String? = nil
    /**
     GET请求
     
     - parameter URLString:  urlString
     - parameter parameters: 参数
     - parameter finished:   完成回调
     */
    func get(_ APIString: String, parameters: [String : Any]?, needHeaders: Bool = true, needLoading: Bool = false, finished: @escaping NetworkFinished) {
        
        
        let headers = needHeaders ? self.headers() : nil
        
        print("\n🌎🌎🌎\nRequest: \nURL: \(rootDomain + APIString)\nMethod: get\nHeaders:\(String(describing: headers))\nParameters: \(String(describing: parameters))\n🌎🌎🌎")
        
        Alamofire.request(rootDomain + APIString, method: .get, parameters: parameters, headers: headers).responseJSON { (response) in
            self.handle(response: response, finished: finished, needLoading: needLoading)
        }
        
    }
    
    /**
     POST请求
     
     - parameter URLString:  urlString
     - parameter parameters: 参数
     - parameter finished:   完成回调
     */
    func post(_ APIString: String, parameters: [String : Any]?, needHeaders: Bool = true, needLoading: Bool = false, finished: @escaping NetworkFinished) {
        
        let headers = needHeaders ? self.headers() : nil
        
        print("\n🌎🌎🌎\nRequest: \nURL: \(rootDomain + APIString)\nMethod: post\nHeaders:\(String(describing: headers))\nParameters: \(String(describing: parameters))\n🌎🌎🌎")
        
        Alamofire.request(rootDomain + APIString, method: .post, parameters: parameters, headers: headers).responseJSON { (response) in
            self.handle(response: response, finished: finished, needLoading: needLoading)
        }
    }
    
    /// 处理响应结果
    ///
    /// - Parameters:
    ///   - response: 响应对象
    ///   - finished: 完成回调
    fileprivate func handle(response: DataResponse<Any>, finished: @escaping NetworkFinished, needLoading: Bool) {
        
        switch response.result {
        case .success(let value):
            let json = JSON(value)
            if json["code"].string == "OK" {
                finished(.success, json, json["message"].string)
            } else {
                finished(.unusual, json, json["message"].string)
            }
        case .failure(let error):
            finished(.failure, nil, error.localizedDescription)
        }
    }
    
    /// 获取请求头
    ///
    /// - Returns: 字典-请求头内容
    fileprivate func headers() -> Dictionary<String, String>! {
        
        let acceptLanguage = Locale.preferredLanguages
            .prefix(6)
            .enumerated()
            .map { index, languageCode in
                let quality = 1.0 - (Double(index) * 0.1)
                return "\(languageCode),q=\(quality)"
            }
            .joined(separator: ", ")
        
        let userAgent: String = {
            if let info = Bundle.main.infoDictionary {
                return "HEXA/iOS/" + (info["CFBundleShortVersionString"] as? String ?? "Unknown")
            }
            return "HEXA/iOS/Unknown"
        }()
        
        return [
            "Host" : "ihome.ngrok.xiaomiqiu.cn",
            "Connection" : "keep-alive",
            "Authorization" : "Basic dXNlcjoxMjM0NQ==",
            "Accept": "application/json"
        ]
    }
    
}
