//
//  NNTabBarController.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.
//

import UIKit

class NNTabBarController: UITabBarController{
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // tabBar 颜色
        tabBar.tintColor = UIColor.init(colorLiteralRed: 0.000, green: 0.757, blue: 0.682, alpha: 1)
        // tabBar 背景图片
        tabBar.backgroundImage = UIImage(named: "pfb_tabbar_background")
        // 添加自控制器
        addChildControllers()
    }
    
    // MARK: - 添加子控制器 NNItemController
    func addChildControllers() {
        addChildController(childController: NNHomeBarController(), title: "Home", imageName: "pfb_tabbar_homepage")
        addChildController(childController: NNItemTableViewController(), title: "Route", imageName: "pfb_tabbar_order")
        addChildController(childController: NNClassificationController(), title: "Message", imageName: "pfb_tabbar_discover")
        addChildController(childController: NNMeController(), title: "Me", imageName: "pfb_tabbar_mine")
    }
    
    // MARK: 添加子控制器
    private func addChildController(childController:UIViewController, title:String, imageName:String) {
        childController.title = title
        childController.tabBarItem.image = UIImage(named: imageName)
        childController.tabBarItem.selectedImage = UIImage(named: imageName + "_selected")
        let navigationVC = NNNavigationController(rootViewController: childController)
        addChildViewController(navigationVC)
    }
}


