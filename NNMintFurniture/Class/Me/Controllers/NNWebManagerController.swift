//
//  NNWebManagerController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/27.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import Foundation

import UIKit

import WebKit

class NNWebManagerController: NNBaseViewController {
    
    var url = "https://www.baidu.com/"
    var webView: WKWebView!
    var webContainerView: UIView!
    override func viewDidLoad() {
        super.viewDidLoad()
        title = "后台管理"
        // Do any additional setup after loading the view.
        webContainerView = UIView()
        webContainerView.frame = CGRect(x:0,y:64,width:self.view.bounds.size.width,height:self.view.bounds.size.height)
        webView = WKWebView(frame: webContainerView.frame)
        // 下面一行代码意思是充满的意思(一定要加，不然也会显示有问题)
        webView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        
        webContainerView.addSubview(webView)
        
        let mapwayURL = URL(string: url)!
        let mapwayRequest = URLRequest(url: mapwayURL)
        webView.load(mapwayRequest)
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
}
