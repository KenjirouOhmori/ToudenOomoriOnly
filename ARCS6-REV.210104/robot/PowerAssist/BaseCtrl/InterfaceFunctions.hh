//! @file InterfaceFunctions.hh
//! @brief インターフェースクラス
//! @date 2020/04/09
//! @author Yokokura, Yuki
//
// Copyright (C) 2011-2020 Yokokura, Yuki
// This program is free software;
// you can redistribute it and/or modify it under the terms of the BSD License.
// For details, see the License.txt file.

#ifndef INTERFACEFUNCTIONS
#define INTERFACEFUNCTIONS

// 基本のインクルードファイル
#include <array>
#include "ConstParams.hh"
#include "ARCSeventlog.hh"
#include "ARCSassert.hh"
#include "ARCSprint.hh"

// 追加のARCSライブラリをここに記述
#include "Limiter.hh"
#include "PCI-6205C.hh"
#include "PCI-3180.hh"
#include "PCI-3310.hh"
#include "PCI-46610x.hh"
#include "WEF-6A.hh"

namespace ARCS {	// ARCS名前空間

//! @brief インターフェースクラス
//! 「電流指令,位置,トルク,…等々」と「DAC,エンコーダカウンタ,ADC,…等々」との対応を指定します。
class InterfaceFunctions {
	public:
		// ここにインターフェース関連の定数を記述する(記述例はsampleを参照)
		// エンコーダ関連の定数
		static constexpr unsigned int PCI6205C_ADDR1 = 0x4080;	// ENC PCI-6205Cのベースアドレス設定
		static constexpr unsigned int PCI6205C_ADDR2 = 0x4060;
		static constexpr unsigned int PCI6205C_ADDR3 = 0x4040;
		static constexpr unsigned int PIC6205C_ADDR4 = 0x4020;
		static constexpr bool EMC_MULT4 = true;					// 4逓倍設定
		static constexpr long ENC_MAX_COUNT = 16386*4;			// [pulse] エンコーダ最大値（逓倍後）Pn202の設定値
		static constexpr int ENC_POLEPARE = 5;					// [-] 極対数 	本当？
		static constexpr double ENC_TO_RADIAN = 2.0*M_PI / ((double)ENC_MAX_COUNT);// [rad/pulse] ラジアンへの換算値

		// 加速度センサ関連の定数
		static constexpr unsigned int PCI3180_ADDR = 0x4000;	// ADC PCI-3180のベースアドレス設定
		static constexpr unsigned int PCI3180_ENA  = 0b00000111;// ADC ENA:CH1, CH2, CH3 DIS:CH4
		static constexpr double ACCSEN_TO_MPS2 = 0.2;			// [(m/s^2)/V] 接線か速度への換算値
		static constexpr double ACCSEN_RADIUS = 0.2;			// [m] 加速度センサ設置半径

		// サーボアンプ関連の定数
		static constexpr unsigned int PCI3310_ADDR = 0x3000;
		static constexpr double SRV_A_TO_V = -3.3 / 2.5;		// [V/A] 電流電圧換算ゲイン	
															// 定格電流 2.5A  サーボアンプの設定 3.3V(Pn400=33) V/定格トルク
		// 力覚センサ関連の定数
		static constexpr unsigned long PCI466102_ADDR = 0xa2102000;	// RS422 PCI-466102のベースアドレス設定		
		std::unique_ptr<PCI46610x> RS422CH1;// RS422シリアル通信ボード
		std::unique_ptr<WEF6A> ForceSensor;	// 6軸力覚センサ スマートポインタというやつらしい　deleteが不要になる。
		
		// ここにD/A，A/D，エンコーダIFボードクラス等々の宣言を記述する(記述例はsampleを参照)
		PCI6205C ENC;
		PCI3180  ADC;
		PCI3310  DAC;

		//! @brief コンストラクタ
		InterfaceFunctions()
			// ここにD/A，A/D，エンコーダIFボードクラス等々の初期化子リストを記述する(記述例はsampleを参照)
			: ENC(PCI6205C_ADDR1, PCI6205C_ADDR2, PCI6205C_ADDR3, PIC6205C_ADDR4, ConstParams::ACTUATOR_NUM, EMC_MULT4),
			  ADC(PCI3180_ADDR, PCI3180::RANGE_B_5V, PCI3180_ENA),
			  DAC(PCI3310_ADDR),
			  ForceSensor(nullptr), RS422CH1(nullptr)
		{
			RS422CH1 = std::make_unique<PCI46610x>(PCI466102_ADDR);	// RS422シリアル通信ボード
			RS422CH1->SetConfig(
				PCI46610x::RATE_921_6kbps,	// ボーレート 921.6kbps
				PCI46610x::WIRE_4,			// 4線式
				PCI46610x::PARITY_DISABLE,	// パリティなし
				PCI46610x::STOPBIT_1,		// ストップビット1
				PCI46610x::WORDLENG_8		// ワード長8bit
			);
			ForceSensor = std::make_unique<WEF6A>(std::move(RS422CH1)); 	//力センサボード この定義ではアロー演算子を使う			
			PassedLog();
		}

		//! @brief デストラクタ
		~InterfaceFunctions(){
			SetZeroCurrent();	// 念のためのゼロ電流指令
			PassedLog();
		}

		//! @brief サーボON信号を送出する関数
		void ServoON(void){
			// ここにサーボアンプへのサーボON信号の送出シーケンスを記述する
			ENC.ZpulseClear(false);		// Z相が来てもクリアしないようにする
		}

		//! @brief サーボOFF信号を送出する関数
		void ServoOFF(void){
			// ここにサーボアンプへのサーボOFF信号の送出シーケンスを記述する
			
		}
		
		//! @brief 電流指令をゼロに設定する関数
		void SetZeroCurrent(void){
			// ここにゼロ電流指令とサーボアンプの関係を列記する
			double V[4]={0};
			DAC.SetVoltage(V);
		}
		
		//! @brief 位置応答を取得する関数
		//! @param[out]	PositionRes	位置応答 [rad]
		void GetPosition(std::array<double, ConstParams::ACTUATOR_NUM>& PositionRes){
			// ここにエンコーダとPosition配列との関係を列記する
			std::array<long, PCI6205C::MAX_CH> count = {0};
			ENC.GetCount(count);	// エンコーダカウント数を取得
			// モータ機械角の取得
			PositionRes[0] = ConvMotorAngle( count[0] );
			PositionRes[1] = ConvMotorAngle( count[1] );
		}
		
		//! @brief 位置応答と速度応答を取得する関数
		//! @param[out]	PositionRes	位置応答 [rad]
		//! @param[out]	VelocityRes	速度応答 [rad/s]
		void GetPositionAndVelocity(
			std::array<double, ConstParams::ACTUATOR_NUM>& PositionRes,
			std::array<double, ConstParams::ACTUATOR_NUM>& VelocityRes
		){
			// ここにエンコーダ，速度演算結果とPositionRes配列，VelocityRes配列との関係を列記する
			
		}
		
		//! @brief モータ電気角と機械角を取得する関数
		//! @param[out]	ElectAngle	電気角 [rad]
		//! @param[out]	MechaAngle	機械角 [rad]
		void GetElectricAndMechanicalAngle(
			std::array<double, ConstParams::ACTUATOR_NUM>& ElectAngle,
			std::array<double, ConstParams::ACTUATOR_NUM>& MechaAngle
		){
			// ここにモータ電気角，機械角とElePosition配列，MecPosition配列との関係を列記する
			std::array<long, PCI6205C::MAX_CH> count = {0};
			ENC.GetCount(count);	// エンコーダカウント数を取得
			// モータ電気角の取得
			ElectAngle[0] = ConvElectAngle( count[0]);	// [rad] 1軸目
			ElectAngle[1] = ConvElectAngle( count[1]);	// [rad] 2軸目
			// モータ機械角の取得
			MechaAngle[0] = ConvMotorAngle( count[0]);	// [rad] 1軸目
			MechaAngle[1] = ConvElectAngle( count[1]);	// [rad] 2軸目
		}
		
		//! @brief トルク応答を取得する関数
		//! @param[out]	Torque	トルク応答 [Nm]
		void GetTorque(std::array<double, ConstParams::ACTUATOR_NUM>& Torque){
			// ここにトルクセンサとTorque配列との関係を列記する
			
		}
		
		//! @brief 加速度応答を取得する関数
		//! @param[out]	Acceleration	加速度応答 [rad/s^2]
		void GetAcceleration(std::array<double, ConstParams::ACTUATOR_NUM>& Acceleration){
			// ここに加速度センサとAcceleration配列との関係を列記する
			
		}
		
		//! @brief トルク応答と加速度応答を取得する関数
		//! @param[out]	Torque	トルク応答 [Nm]
		//! @param[out]	Acceleration	加速度応答 [rad/s^2]
		void GetTorqueAndAcceleration(
			std::array<double, ConstParams::ACTUATOR_NUM>& Torque,
			std::array<double, ConstParams::ACTUATOR_NUM>& Acceleration
		){
			// ここにトルクセンサとTorque配列との関係，加速度センサとAcceleration配列との関係を列記する
			double V1, V2, V3, V4;			// [V] 入力電圧
			ADC.ConvStart();				// AD変換開始
			ADC.WaitBusy();					// AD変換が完了するまで待機（ブロッキング動作）
			ADC.GetVoltage(V1, V2, V3, V4);	// [V] 電圧値の取得
			Torque[0] = V1;					// 代入は適当に書きました by 大森
			Torque[1] = V2;
			Acceleration[0] = V3;
		}
		
		//! @brief 電流応答を取得する関数
		//! @param[out]	Current	電流応答 [A]
		void GetCurrent(std::array<double, ConstParams::ACTUATOR_NUM>& Current){
			// ここに電流センサとCurrent配列との関係を列記する
			
		}
		
		//! @brief 電流指令を設定する関数
		//! @param[in]	Current	電流指令 [A]
		void SetCurrent(const std::array<double, ConstParams::ACTUATOR_NUM>& Current){
			// ここにCurrent配列とサーボアンプの関係を列記する
			double Vout[PCI3310::MAX_CH]={0};
			Vout[0] = SRV_A_TO_V*Current[0];	// CH1
			Vout[1] = SRV_A_TO_V*Current[1];	// CH2
			Vout[2] = SRV_A_TO_V*Current[2];	// CH3
			Vout[3] = SRV_A_TO_V*Current[3];	// CH4
			DAC.SetVoltage(Vout);	// DA変換電圧更新
		}
		
		//! @brief トルク指令を設定する関数
		//! @param[in]	Torque	トルク指令 [Nm]
		void SetTorque(const std::array<double, ConstParams::ACTUATOR_NUM>& Torque){
			// ここにTorque配列とサーボアンプの関係を列記する
			
		}
		//! @brief 6軸力覚センサの信号送信を命令する関数
		void Order6axisForce(){
			// ここに6軸力覚センサとForce配列との関係を列記する
			ForceSensor->SendForceRequest();	// リクエスト送信
		}


		//! @brief 6軸力覚センサ応答を取得する関数
		//! @param[out]	Fx-Fy 各軸の並進力 [N]
		//! @param[out]	Mx-My 各軸のトルク [Nm]
		void Get6axisForce(double& Fx, double& Fy, double& Fz, double& Mx, double& My, double& Mz){
			// ここに6軸力覚センサと各変数との関係を列記する
			ForceSensor->SendForceRequest();	// リクエスト送信
			ForceSensor->WaitForceData();		// データ取得待機(ブロッキング動作)
			ForceSensor->Get6axisForce(Fx, Fy, Fz, Mx, My, Mz);	// [N],[Nm] 力覚センサ値の取得
		}
		
		//! @brief 6軸力覚センサの信号送信を命令する関数
		void ZeroCalibration(){
			// ここに6軸力覚センサとForce配列との関係を列記する
			ForceSensor->ZeroCalibration();	// キャリブレーション実施
		}


		//! @brief 安全装置への信号出力を設定する関数
		//! @param[in]	Signal	安全装置へのディジタル信号
		void SetSafetySignal(const uint8_t& Signal){
			// ここに安全信号とDIOポートとの関係を列記する
			
		}
		
		//! @brief Z相クリアに関する設定をする関数
		//! @param[in]	ClearEnable	true = Z相が来たらクリア，false = クリアしない
		void SetZpulseClear(const bool ClearEnable){
			// インクリメンタルエンコーダのZ(I,C)相クリアの設定が必要な場合に記述する
			ENC.ZpulseClear(ClearEnable); 
		}
		
	private:
		InterfaceFunctions(const InterfaceFunctions&) = delete;					//!< コピーコンストラクタ使用禁止
		const InterfaceFunctions& operator=(const InterfaceFunctions&) = delete;//!< 代入演算子使用禁止
		
		// ここにセンサ取得値とSI単位系の間の換算に関する関数を記述(記述例はsampleを参照)
		
		//! @brief モータ機械角 [rad] へ換算する関数
		//! @brief	count	エンコーダカウント値
		//! @return	機械角 [rad]
		static double ConvMotorAngle(const long count){
			return ENC_TO_RADIAN*(double)count;
		}
		
		//! @brief モータ電気角 [rad] へ換算する関数 (-2π～+2πの循環値域制限あり)
		//! @brief	count	エンコーダカウント値
		//! @return	電気角 [rad]
		static double ConvElectAngle(const long count){
			return ENC_TO_RADIAN*(double)(ENC_POLEPARE*( count % (ENC_MAX_COUNT/ENC_POLEPARE) ));
		}
};
}

#endif