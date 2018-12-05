//
//  HTTPSManager.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/19.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation

import UIKit
import Alamofire

class HTTPSManager: NSObject {
    
    func setAlamofireHttps() {
        
        SessionManager.default.delegate.sessionDidReceiveChallenge = { (session: URLSession, challenge: URLAuthenticationChallenge) in
            
            let method = challenge.protectionSpace.authenticationMethod
            
            if method == NSURLAuthenticationMethodServerTrust {
                
                //验证服务器，直接信任或者验证证书二选一，推荐验证证书，更安全
                //return HTTPSManager.trustServerWithCer(challenge: challenge)
                print("HTTPSManager.trustServer  1")
                return HTTPSManager.trustServer(challenge: challenge)
                
            } else if method == NSURLAuthenticationMethodClientCertificate {
                print("HTTPSManager.trustServer  2")
                //认证客户端证书
                return HTTPSManager.sendClientCer()
                
            } else {
                print("HTTPSManager.trustServer  2")
                //其他情况，不通过验证
                return (.cancelAuthenticationChallenge, nil)
                
            }
            
        }
        
    }
    
    //不做任何验证，直接信任服务器
    static private func trustServer(challenge: URLAuthenticationChallenge) -> (URLSession.AuthChallengeDisposition, URLCredential?) {
        
        let disposition = URLSession.AuthChallengeDisposition.useCredential
        let credential = URLCredential.init(trust: challenge.protectionSpace.serverTrust!)
        return (disposition, credential)
        
    }
    
    //验证服务器证书
    static private func trustServerWithCer(challenge: URLAuthenticationChallenge) -> (URLSession.AuthChallengeDisposition, URLCredential?) {
        
        var disposition: URLSession.AuthChallengeDisposition = .performDefaultHandling
        var credential: URLCredential?
        
        //获取服务器发送过来的证书
        let serverTrust:SecTrust = challenge.protectionSpace.serverTrust!
        let certificate = SecTrustGetCertificateAtIndex(serverTrust, 0)!
        let remoteCertificateData = CFBridgingRetain(SecCertificateCopyData(certificate))!
        
        //加载本地CA证书
        let cerPath = Bundle.main.path(forResource: "server", ofType: "cer")!
        let cerUrl = URL(fileURLWithPath:cerPath)
        let localCertificateData = try! Data(contentsOf: cerUrl)
        
        if (remoteCertificateData.isEqual(localCertificateData) == true) {
            
            //服务器证书验证通过
            disposition = URLSession.AuthChallengeDisposition.useCredential
            credential = URLCredential(trust: serverTrust)
            
        } else {
            
            //服务器证书验证失败
            disposition = URLSession.AuthChallengeDisposition.cancelAuthenticationChallenge
            
        }
        
        return (disposition, credential)
        
    }
    
    //发送客户端证书交由服务器验证
    static private func sendClientCer() -> (URLSession.AuthChallengeDisposition, URLCredential?) {
        
        let disposition = URLSession.AuthChallengeDisposition.useCredential
        var credential: URLCredential?
        
        //获取项目中P12证书文件的路径
        let path: String = Bundle.main.path(forResource: "你本地的p12证书文件名", ofType: "p12")!
        let PKCS12Data = NSData(contentsOfFile:path)!
        let key : NSString = kSecImportExportPassphrase as NSString
        let options : NSDictionary = [key : "p12证书的密码"] //客户端证书密码
        
        var items: CFArray?
        let error = SecPKCS12Import(PKCS12Data, options, &items)
        
        if error == errSecSuccess {
            
            let itemArr = items! as Array
            let item = itemArr.first!
            
            let identityPointer = item["identity"];
            let secIdentityRef = identityPointer as! SecIdentity
            
            let chainPointer = item["chain"]
            let chainRef = chainPointer as? [Any]
            
            credential = URLCredential.init(identity: secIdentityRef, certificates: chainRef, persistence: URLCredential.Persistence.forSession)
            
        }
        return (disposition, credential)
    }
}


//public func defaultAlamofireManager() -> Manager {
//    let configuration = URLSessionConfiguration.default
//    configuration.httpAdditionalHeaders = Alamofire.SessionManager.defaultHTTPHeaders
//    
//    let policies: [String: ServerTrustPolicy] = [
//        
//        "api.domian.cn": .disableEvaluation
//    ]
//    let manager = Alamofire.SessionManager(configuration: configuration,serverTrustPolicyManager: ServerTrustPolicyManager(policies: policies))
//    
//    manager.startRequestsImmediately = false
//    return manager
//}
//在程序启动进行网络请求之前，调用一次setAlamofireHttps方法就好了。
