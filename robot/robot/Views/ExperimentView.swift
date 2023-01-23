//
//  ExperimentView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 10/05/2022.
//

import SwiftUI

struct ExperimentView: View {
    @EnvironmentObject var rl: RL

    var body: some View {
        VStack {
            Divider()
            ChartView()
            Divider()
            RunningView()
            Divider()
        }

    }
}
