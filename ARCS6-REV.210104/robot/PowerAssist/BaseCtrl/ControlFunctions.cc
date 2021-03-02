//! @file ControlFunctions.cc
//! @brief 制御用周期実行関数群クラス
//! @date 2020/04/09
//! @author Yokokura, Yuki
//
// Copyright (C) 2011-2020 Yokokura, Yuki
// This program is free software;
// you can redistribute it and/or modify it under the terms of the BSD License.
// For details, see the License.txt file.

// 基本のインクルードファイル
#include <unistd.h>
#include <cmath>
#include <cfloat>
#include <tuple>
#include "ControlFunctions.hh"
#include "ARCSprint.hh"
#include "ARCSassert.hh"
#include "ScreenParams.hh"
#include "InterfaceFunctions.hh"
#include "GraphPlot.hh"
#include "DataMemory.hh"

// 追加のARCSライブラリをここに記述
#include "Matrix.hh"
#include "SpeedCalculator.hh"
#include "Limiter.hh"
#include "LowPassFilter.hh"
#include "DisturbanceObsrv.hh"

using namespace ARCS;

//! @brief スレッド間通信用グローバル変数の無名名前空間
namespace {
	// スレッド間で共有したい変数をここに記述
	std::array<double, ConstParams::ACTUATOR_NUM> PositionRes = {0};	//!< [rad] 位置応答
	std::array<double, ConstParams::ACTUATOR_NUM> CurrentRef = {0};		//!< [Nm]  電流指令
	std::array<double, ConstParams::ACTUATOR_NUM> VelocityRes = {0};	//!< [rad/s] 速度応答
	std::array<double, ConstParams::ACTUATOR_NUM> VelocityRef = {0};	//!< [rad/s] 速度指令
}

//! @brief 制御用周期実行関数1
//! @param[in]	t		時刻 [s]
//! @param[in]	Tact	計測周期 [s]
//! @param[in]	Tcmp	消費時間 [s]
//! @return		クロックオーバーライドフラグ (true = リアルタイムループ, false = 非リアルタイムループ)
bool ControlFunctions::ControlFunction1(double t, double Tact, double Tcmp){
	// 制御用定数設定
	[[maybe_unused]] constexpr double Ts = ConstParams::SAMPLING_TIME[0]*1e-9;	// [s]	制御周期
	
	// 機械パラメータ
	const double Rg = 66;	// 減速日
	const double r = 0.24;	// [m] 車輪半径
	const double M = 189;	// [kg] 車体重量
	const double Kt = 0.544;	// [Nm/A] トルク定数
	const double Jw = 2.575e-4;	// [kgm^2] 慣性モーメント

	// 制御用変数宣言
	static SpeedCalculator<10> SpdCalc_r;	// [rad/s] 右側駆動輪速度　サンプリング数 10
	static SpeedCalculator<10> SpdCalc_l;	// [rad/s] 左側駆動輪速度　サンプリング数 10
	static SpeedCalculator<2>  AccCalc;		// [m/s^2] 加速度　サンプリング数 2
	static double aveVelocityRes = 0;	// [m/s] 左右輪の平均速度
	static double aveAcceleRes = 0;		// [m/s^2] 平均加速度
	static double Fsenlpf = 0;			// [N] Z軸センサ値＋LPF
	static double avePositionRes = 0;		// [m] 左右輪平均位置
	static double RawForceData[6] = {0};		// [N][Nm] 6軸力覚センサの格納用
	static double tau_dis = 0;	// [Nm] モータ側に加わる外乱トルク
	static double Fdis = 0;		// [N] 機体に加わる外乱
	const double CurrentMax = 2.5*3;		// [A] 電流指令の制限 定格電流の3倍

	const double MaxOffset = 1/Ts;	// 最大オフセット取得個数　経過時間 / サンプリング周期
	static double Foffset = 0;		// [N] 力覚センサ用オフセット
	static double OfsCount = 0;		// オフセット取得カウント用

	// 制御器パラメータ
	const double gdis = 20;				// [rad/s] 外乱オブザーバの帯域
	const double glpf = 20;				// [rad/s] 加速度LPF帯域
	const double Mn = M + 2*Jw*(Rg*Rg)/(r*r);	// [kg] ノミナル慣性
	const double Ktn = Kt;				// [Nm/A] ノミナルトルク定数
	const double Ct = 1/(Mn/20);		// 力制御ゲイン

	// 制御器宣言
	static LowPassFilter *DOBLPF;	// DOB用
	static LowPassFilter *FsenLPF;		// 加速度用
	//static DisturbanceObsrv DOB(2*Ktn, Mn, gdis, Ts);

	if(CmdFlag == CTRL_INIT){
		// 初期化モード (ここは制御開始時/再開時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
		Initializing = true;		// 初期化中ランプ点灯
		Screen.InitOnlineSetVar();	// オンライン設定変数の初期値の設定
		DOBLPF = new LowPassFilter(gdis, Ts);
		FsenLPF = new LowPassFilter(glpf, Ts);
		Interface.ServoON();		// サーボON指令の送出
		Initializing = false;		// 初期化中ランプ消灯
		// 念のために、サーボアンプの電流指令は0とする
		CurrentRef[0] = 0.0;
		CurrentRef[1] = 0.0;
		CurrentRef[2] = 0.0;
		CurrentRef[3] = 0.0;
	}
	if(CmdFlag == CTRL_LOOP){
		// 周期モード (ここは制御周期 SAMPLING_TIME[0] 毎に呼び出される(リアルタイム空間なので処理は制御周期内に収めること))
		// リアルタイム制御ここから
		Interface.GetPosition(PositionRes);	// [rad] 位置応答の取得
		avePositionRes = (PositionRes[0]+PositionRes[1])/2*r/Rg;	// [m] 平均位置
		Screen.GetOnlineSetVar();			// オンライン設定変数の読み込み
		Interface.Get6axisForce(RawForceData[0], RawForceData[1], RawForceData[2], RawForceData[3], RawForceData[4], RawForceData[5]);	// [N],[Nm] 力覚センサ値の取得
		VelocityRes[0] = SpdCalc_r.GetSpeed(PositionRes[0], t);	// 右側駆動輪回転数計算
		VelocityRes[1] = SpdCalc_l.GetSpeed(PositionRes[1], t); // 左側駆動輪回転数計算
		aveVelocityRes = r / Rg * (VelocityRes[0] + VelocityRes[1]) /2;
		aveAcceleRes = AccCalc.GetSpeed(aveVelocityRes, t);
		Fsenlpf = FsenLPF->GetSignal(RawForceData[2]);
		// ここに制御アルゴリズムを記述する
		VelocityRef[0] = 0;
		VelocityRef[1] = 0; 
		// DOB
		Fdis = DOBLPF->GetSignal(CurrentRef[0]*2*Ktn*Rg/r + Mn*gdis*aveVelocityRes) - Mn*gdis*aveVelocityRes; 

		CurrentRef[0] = RawForceData[2]*Ct*Mn/(2*Ktn)*r/Rg + Fdis/(2*Ktn)*r/Rg;
		CurrentRef[1] = RawForceData[2]*Ct*Mn/(2*Ktn)*r/Rg + Fdis/(2*Ktn)*r/Rg;		

		CurrentRef[0] = Limiter(CurrentRef[0], CurrentMax);
		CurrentRef[1] = Limiter(CurrentRef[1], CurrentMax);		
		Interface.SetCurrent(CurrentRef);	// [A] 電流指令の出力
		Screen.SetVarIndicator(Mn, gdis, OfsCount, Foffset, RawForceData[2], 0, 0, 0, 0, 0);	// 任意変数インジケータ(変数0, ..., 変数9)
		Graph.SetTime(Tact, t);									// [s] グラフ描画用の周期と時刻のセット
		Graph.SetVars(0, RawForceData[2], Fdis, 0, 0, 0, 0, 0, 0);	// グラフプロット0 (グラフ番号, 変数0, ..., 変数7)
		Graph.SetVars(1, CurrentRef[0], 0, 0, 0, 0, 0, 0, 0);	// グラフプロット1 (グラフ番号, 変数0, ..., 変数7)
		Graph.SetVars(2, avePositionRes, 0, 0, 0, 0, 0, 0, 0);	// グラフプロット2 (グラフ番号, 変数0, ..., 変数7)
		Graph.SetVars(3, aveVelocityRes, 0, 0, 0, 0, 0, 0, 0);	// グラフプロット3 (グラフ番号, 変数0, ..., 変数7)
		Memory.SetData(Tact, t, CurrentRef[0], avePositionRes, aveVelocityRes, aveAcceleRes, Fdis, RawForceData[2], RawForceData[1], RawForceData[3], Fsenlpf);		// CSVデータ保存変数 (周期, A列, B列, ..., J列)
		// リアルタイム制御ここまで
	}
	if(CmdFlag == CTRL_EXIT){
		// 終了処理モード (ここは制御終了時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
		Interface.SetZeroCurrent();	// 電流指令を零に設定
		Interface.ServoOFF();		// サーボOFF信号の送出
	}
	return true;	// クロックオーバーライドフラグ(falseにすると次の周期時刻を待たずにスレッドが即刻動作する)
}

//! @brief 制御用周期実行関数2
//! @param[in]	t		時刻 [s]
//! @param[in]	Tact	計測周期 [s]
//! @param[in]	Tcmp	消費時間 [s]
//! @return		クロックオーバーライドフラグ (true = リアルタイムループ, false = 非リアルタイムループ)
bool ControlFunctions::ControlFunction2(double t, double Tact, double Tcmp){
	// 制御用定数宣言
	[[maybe_unused]] const double Ts = ConstParams::SAMPLING_TIME[1]*1e-9;	// [s]	制御周期
	
	// 制御用変数宣言
	
	// 制御器等々の宣言
	
	if(CmdFlag == CTRL_INIT){
		// 初期化モード (ここは制御開始時/再開時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
	}
	if(CmdFlag == CTRL_LOOP){
		// 周期モード (ここは制御周期 SAMPLING_TIME[1] 毎に呼び出される(リアルタイム空間なので処理は制御周期内に収めること))
		// リアルタイム制御ここから
		
		// リアルタイム制御ここまで
	}
	if(CmdFlag == CTRL_EXIT){
		// 終了処理モード (ここは制御終了時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
	}
	return true;	// クロックオーバーライドフラグ(falseにすると次の周期時刻を待たずにスレッドが即刻動作する)
}

//! @brief 制御用周期実行関数3
//! @param[in]	t		時刻 [s]
//! @param[in]	Tact	計測周期 [s]
//! @param[in]	Tcmp	消費時間 [s]
//! @return		クロックオーバーライドフラグ (true = リアルタイムループ, false = 非リアルタイムループ)
bool ControlFunctions::ControlFunction3(double t, double Tact, double Tcmp){
	// 制御用定数宣言
	[[maybe_unused]] const double Ts = ConstParams::SAMPLING_TIME[2]*1e-9;	// [s]	制御周期
	
	// 制御用変数宣言
	
	if(CmdFlag == CTRL_INIT){
		// 初期化モード (ここは制御開始時/再開時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
	}
	if(CmdFlag == CTRL_LOOP){
		// 周期モード (ここは制御周期 SAMPLING_TIME[2] 毎に呼び出される(リアルタイム空間なので処理は制御周期内に収めること))
		// リアルタイム制御ここから
		
		// リアルタイム制御ここまで
	}
	if(CmdFlag == CTRL_EXIT){
		// 終了処理モード (ここは制御終了時に1度だけ呼び出される(非リアルタイム空間なので重い処理もOK))
	}
	return true;	// クロックオーバーライドフラグ(falseにすると次の周期時刻を待たずにスレッドが即刻動作する)
}

//! @brief 制御用変数値を更新する関数
void ControlFunctions::UpdateControlValue(void){
	// ARCS画面パラメータに値を書き込む
	Screen.SetNetworkLink(NetworkLink);						// ネットワークリンクフラグを書き込む
	Screen.SetInitializing(Initializing);					// ロボット初期化フラグを書き込む
	Screen.SetCurrentAndPosition(CurrentRef, PositionRes);	// 電流指令と位置応答を書き込む
}

