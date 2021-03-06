//
//  NNHomePageController.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.
//
import UIKit

class NNHomeBarController: UITabBarController{
    var BDMap_View:NNBaiduMapViewController!
    var HDMap_View:NNHDMapViewController!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // tabBar 颜色
        tabBar.tintColor = UIColor.init(colorLiteralRed: 0.000, green: 0.757, blue: 0.682, alpha: 1)
        // tabBar 背景图片
        tabBar.backgroundImage = UIImage(named: "pfb_tabbar_background")
        
        BDMap_View = NNBaiduMapViewController()
        BDMap_View.socket = gsocket
        HDMap_View = NNHDMapViewController()
        HDMap_View.socket = gsocket

        // 添加自控制器
        addChildControllers()
        
        //创建定时任务
        _ = Timer.scheduledTimer(timeInterval: 3,target:self,selector:#selector(NNHomeBarController.timerFireMethod),
                                 userInfo:nil,repeats:true)
    }
    
    // 参数: The timer passes itself as the argument
    func timerFireMethod() {
        
        if(process == true){
            self.selectedIndex = 1
        }else{
           self.selectedIndex = 0
        }
        
//        if(self.BDMap_View.baidu_index == 1 ){
//            self.selectedIndex = 1
//            //let route : [String] = ["1", "2", "3", "5", "8"]
//            //self.HDMap_View.updateMapWithRoute(route: route)
//            self.BDMap_View.baidu_index = 0
//        }
    }
    // MARK: - 添加子控制器
    func addChildControllers() {
        addChildController(childController: self.BDMap_View, title: "Map", imageName: "pin")
        addChildController(childController: self.HDMap_View, title: "HDMap", imageName: "pin")
    }

    // MARK: 添加子控制器
    private func addChildController(childController:UIViewController, title:String, imageName:String) {
        childController.title = title
        childController.tabBarItem.image = UIImage(named: imageName)
        childController.tabBarItem.selectedImage = UIImage(named: imageName + "_red")
        let navigationVC = NNNavigationController(rootViewController: childController)
        addChildViewController(navigationVC)
    }
}

class NNHomePageController: NNBaseViewController, UIScrollViewDelegate {
    
    weak var contentView = UIScrollView()
    // 当前选中的按钮
    weak var selectedButton = UIButton()
    let titlesArray = ["Map","HDMap"]

    override func viewDidLoad() {
        super.viewDidLoad()
        // 添加控制器
        addChildVC()
        // 导航栏
        setupNav()
        // 顶部标签栏
        setupTitlesView()
        // 主要内容视图
        setupContentView()
    }
    
    // MARK: - 设置 UI 部分
    // MARK: 添加控制器
    func addChildVC() {
        //for _ in 0..<titlesArray.count {
            let vc1 = NNBaiduMapViewController()
            addChildViewController(vc1)
            let vc2 = NNHDMapViewController()
            addChildViewController(vc2)
        //}
    }

    // MARK: 设置导航栏
    func setupNav() {
        navigationItem.rightBarButtonItem = UIBarButtonItem(image:UIImage(named: "icon_homepage_searchbar_left"), style: .plain, target: self, action: #selector(rightBarButtonClick))
    }
    
    // MARK: 顶部标签栏
    func setupTitlesView() {
        view.addSubview(bgView)
        bgView.addSubview(titlesView)
        bgView.addSubview(arrowButton)
        
        // 内部子标签
        let width = titlesView.frame.size.width / CGFloat(titlesArray.count)
        let height = titlesView.frame.size.height
        
        for index in 0..<titlesArray.count {
            let button = UIButton()
            button.frame.size.height = height
            button.frame.size.width = width
            button.frame.origin.x = CGFloat(index) * width
            button.tag = index
            button.titleLabel!.font = UIFont.systemFont(ofSize: 15)
            button.setTitle(titlesArray[index], for: .normal)
            button.setTitleColor(UIColor.gray, for: .normal)
            button.addTarget(self, action: #selector(titlesClick), for: .touchUpInside)
            titlesView.addSubview(button)
            
            if index == 0 {
                button.isEnabled = false
                selectedButton = button
                selectedButton?.setTitleColor(UIColor.red, for: .normal)
                
                // 让按钮内部的 Label 根据文字来计算内容
                button.titleLabel?.sizeToFit()
            }
        }
    }
    
    // MARK: 主要内容视图
    func setupContentView() {
        //automaticallyAdjustsScrollViewInsets = false
        let contentView = UIScrollView()
        contentView.frame = view.bounds
        contentView.delegate = self
        contentView.contentSize = CGSize(width: contentView.frame.size.width * CGFloat(childViewControllers.count), height: 0)
        contentView.isPagingEnabled = true
        contentView.isScrollEnabled = true
        view.insertSubview(contentView, at: 0)
        self.contentView = contentView
        // 添加第一个控制器的 view
        scrollViewDidEndScrollingAnimation(contentView)
    }
    
    // MARK: -  点击按钮事件
    // MARK: 点击了标签栏
    func titlesClick(button: UIButton) {
        selectedButton!.isEnabled = true
        selectedButton!.setTitleColor(UIColor.gray, for: .normal)
        button.isEnabled = false
        selectedButton = button
        selectedButton?.setTitleColor(UIColor.red, for: .normal)
        
        var offset = contentView!.contentOffset
        offset.x = CGFloat(button.tag) * (contentView?.frame.size.width)!
        contentView!.setContentOffset(offset, animated: true)
    }
    
    // MARK: 点击了右边搜索框
    func rightBarButtonClick() {
        navigationController?.pushViewController(NNSearchController(), animated: true)
    }
    
    // MARK: 点击了箭头
    func arrowButtonClick(button: UIButton) {
        UIView.animate(withDuration: 0.25) {
            button.imageView?.transform = button.imageView!.transform.rotated(by: CGFloat(Double.pi))
        }
    }
    
    // MARK: - UIScrollViewDelegate 代理
    // MARK: scrollViewDidEndScrollingAnimation
    func scrollViewDidEndScrollingAnimation(_ scrollView: UIScrollView) {
        let index = Int(scrollView.contentOffset.x / scrollView.frame.size.width)
        // 取出子控制器
        let vc = childViewControllers[index]
        vc.view.frame.origin.x = scrollView.contentOffset.x
        vc.view.frame.origin.y = 0
        // 设置控制器的 view 的 height 值为整个屏幕的高度
        vc.view.frame.size.height = scrollView.frame.size.height
        scrollView.addSubview(vc.view)
    }
    
    // MARK: scrollViewDidEndDecelerating
    func scrollViewDidEndDecelerating(_ scrollView: UIScrollView) {
        scrollViewDidEndScrollingAnimation(scrollView)
        // 当前索引
        let index = Int(scrollView.contentOffset.x / scrollView.frame.size.width)
        // 点击 Button
        let button = titlesView.subviews[index] as! UIButton
        titlesClick(button: button)
    }
    
    // MARK: - 懒加载
    // MARK: 懒加载头部试图背景
    private lazy var bgView: UIView = {
        let bgView = UIView()
        bgView.frame = CGRect(x: 0, y: kTitlesViewY, width: NNScreenWidth, height: kTitlesViewH)
        bgView.backgroundColor = UIColor.white
        return bgView
    }()
    
    // MARK: 懒加载头部视图左边几个按钮
    private lazy var titlesView: UIView = {
        let titlesView = UIView()
        titlesView.frame = CGRect(x: 0, y: 0, width: NNScreenWidth - kTitlesViewH, height: kTitlesViewH)
        return titlesView
    }()
    
    // MARK: 懒加载头部视图右部按钮
    private lazy var arrowButton: UIButton = {
        let arrowButton = UIButton()
        arrowButton.frame = CGRect(x: NNScreenWidth - kTitlesViewH, y: 0, width: kTitlesViewH, height: kTitlesViewH)
        arrowButton.setImage(UIImage(named: "icon_ordercenter_arrow_down"), for: .normal)
        arrowButton.addTarget(self, action: #selector(arrowButtonClick), for: .touchUpInside)
        return arrowButton
    }()
}

