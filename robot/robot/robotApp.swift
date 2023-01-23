//
//  robotApp.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 30/03/2022.
//

import SwiftUI

@main
struct robotApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) var appDelegate
    @StateObject var rl = RL()
    var body: some Scene {
        WindowGroup {
            if rl.view == 1 {
                ExperimentView().environmentObject(rl)
            } else if (rl.view == 2){
                NewView().environmentObject(rl)
            } else if (rl.view == 3){
                PIDView().environmentObject(rl)
            }else if (rl.view == 4){
                MocapView().environmentObject(rl)
            }else if (rl.view == 5){
                ManView().environmentObject(rl)
            }else if (rl.view == 101){
                NetworkView().environmentObject(rl)
            }else if (rl.view == 7){
                ExhaustiveView().environmentObject(rl)
            }else if (rl.view == 102){
                TrimView().environmentObject(rl)
            } else {
                HomeView().environmentObject(rl)
            }
        }
    }
}

class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        return true
    }
}
