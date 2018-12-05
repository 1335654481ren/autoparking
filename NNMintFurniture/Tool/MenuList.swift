//
//  MenuList.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/11/17.
//  Copyright © 2018年 zhongNing. All rights reserved.
//
import Foundation
import UIKit

class Menu: UIView,UITableViewDelegate,UITableViewDataSource {
    /*
     menu 的table 和 内容
     */
    var menuTab: UITableView?
    var menuArr : Array<Any>?
    let cellIdentifier = "cellID"
    /*
     menu 选中行的方法
     */
    var didSelectIndex:((_ index:Int)->Void)?
    /*
     加载动画效果
     */
    var isShow : Bool?
    var menuSize : CGSize?
    
    class func initMenu(size:CGSize)->Menu{
        
        let frame = CGRect(x:0,y:0,width:size.width,height:size.height)
        let me = Menu.init(frame:frame)
        me.menuSize = size
        return me
    }
    
    
    override init(frame: CGRect) {
        
        /**
         这时候 frame 的 height 设置为 0 ，是为了加载缓慢弹出的动画……
         */
        let  initialFrame = CGRect(x:frame.origin.x,y:frame.origin.y,width:frame.size.width,height:0.0)
        
        super.init(frame: initialFrame)
        self.backgroundColor = UIColor.black
        
        menuTab = UITableView.init(frame: CGRect(x:0,y:0,width:frame.size.width,height:0), style: .plain)
        menuTab?.tableFooterView = UIView.init()
        menuTab?.delegate = self
        menuTab?.dataSource = self
        
        addSubview(menuTab!)
        
        menuTab?.isHidden = true
        isHidden = true
        isShow = false
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    /*
     menu 弹出菜单的方法
     */
    func popupMenu(orginPoint:CGPoint,arr : Array<Any>){
        
        if self.isShow == true {
            return
        }
        self.isShow = true
        self.isHidden = false
        menuTab?.isHidden = false
        
        
        self.frame.origin = orginPoint
        self.menuArr = arr
        self.menuTab?.reloadData()
        
        self.superview?.bringSubview(toFront: self)
        menuTab?.frame.size.height = 0.0
        /**
         这里是 弹出的动画
         */
        UIView.animate(withDuration: 0.5, animations: {
            
            self.frame.size.height = (self.menuSize!.height)
            self.menuTab?.frame.size.height = (self.menuSize!.height)
            
        }) { (finish) in
        }
    }
    /**
     这里是收回菜单的方法
     */
    
    func packUpMenu() {
        
        if self.isShow == false {
            return
        }
        self.isShow = false
        
        UIView.animate(withDuration: 0.5, animations: {
            
            self.menuTab?.frame.size.height = 0.0
            self.frame.size.height = 0.0
            
        }) { (finish) in
            
            self.isHidden = true
            self.menuTab?.isHidden = true
        }
    }
    
    /*
     tableView delegate dataSource
     */
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        
        if ((menuArr?.count) != nil) {
            return (menuArr?.count)!
        }
        return 1
    }
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell{
        
        var cell = tableView.dequeueReusableCell(withIdentifier: cellIdentifier)
        if cell == nil {
            cell = UITableViewCell.init(style: .default, reuseIdentifier: cellIdentifier)
        }
        cell?.textLabel?.text = menuArr?[indexPath.row] as? String
        cell?.textLabel?.textAlignment = .center
        cell?.textLabel?.font = UIFont.systemFont(ofSize: 16.0)
        
        return cell!
    }
    
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat{
        return 30
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath){
        
        tableView.deselectRow(at: indexPath, animated: true)
        /**
         这里是把 选中的 indexPath 传值出去 ， 关闭menu列表
         */
        if (self.didSelectIndex != nil) {
            self.didSelectIndex!(indexPath.row)
        }
        self.packUpMenu()
    }
}
