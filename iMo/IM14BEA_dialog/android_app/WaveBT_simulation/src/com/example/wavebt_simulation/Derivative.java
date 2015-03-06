package com.example.wavebt_simulation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.os.Message;



public class Derivative {
	private int sample_rate = 100;	//采样率
	private int N_buf;	//一个数据帧的长度
	private int d1,d2;//一阶、二阶差分值
	private int sum;//一阶二阶差分值相加
	private Derivator derivator1 = new Derivator();	//获得一阶微分器、平滑器实例
	private Smoother smoother1 = new Smoother();
	private Derivator derivator2 = new Derivator();	//获得二阶微分器、平滑器实例
	private Smoother smoother2 = new Smoother();
	private int THRESHOLD_UP;	//迟滞比较器的一组阈值
	private int THRESHOLD_DOWN;
	private int RR_counter;	//RR间期计数器
	private int RR_interval;	//RR间期
	private int QRS_counter;	//QRS间期计数器
	private int QRS_duration;	//QRS间期
	private int QRS_state;	//QRS状态机
	private int[] RR_intervals = new int[8];	//存放RR间期数组，用来计算心率
	private int heart_rate;	//心率
	private int RR_intervals_counter;	//RR间期数组的索引
	private boolean THRESHOLD_INITED;	//阈值初始化完成标志
	private boolean START;	//开始计算RR间期标志位
	private Arrhythmia arrhythmia = new Arrhythmia();
	
	
	/*
	 * 微分器
	 */
	private class Derivator{
		private int x,x1,x2;
		private int get_derivative(int data){
			int y0;
			x = data;
			y0 = x - x2;
			x2 = x1;
			x1 = x;
			return y0;
		}
	}
	
	/*
	 * 平滑器
	 */
	private class Smoother{
		private int x, x1, x2;
		private int smooth(int data){
			int y0;
			x = data;
			y0 = (x + 2*x1 + x2);
			x2 = x1;
			x1 = x;
			return y0;
		}
	}
	
	/*
	 * 显示信息
	 */
	private void display(){
		//System.out.print("RR:"+RR_interval+"  QRS:"+QRS_duration+"  ");
		//System.out.print("HEART RATE:"+heart_rate+"\n");
		//向handler发送message
		Message msg_rr = WaveActivity.handler.obtainMessage();
		msg_rr.what = 1;
		msg_rr.arg1 = RR_interval;
		WaveActivity.handler.sendMessage(msg_rr);
		Message msg_qrs = WaveActivity.handler.obtainMessage();
		msg_qrs.what = 2;
		msg_qrs.arg1 = QRS_duration;
		WaveActivity.handler.sendMessage(msg_qrs);
		Message msg_heartrate = WaveActivity.handler.obtainMessage();
		msg_heartrate.what = 3;
		msg_heartrate.arg1 = heart_rate;
		WaveActivity.handler.sendMessage(msg_heartrate);
		Message msg_arrhythmia = WaveActivity.handler.obtainMessage();
		msg_arrhythmia.what = 4;
		msg_arrhythmia.arg1 = arrhythmia.abnormal_count;
		WaveActivity.handler.sendMessage(msg_arrhythmia);
	}
	
	/*
	 * 计算心率
	 */
	private void calculate_heart_rate(){
		RR_intervals[RR_intervals_counter] = RR_interval;
		if(++RR_intervals_counter == 8)
			RR_intervals_counter = 0;
		//如果存满8个RR间期，开始计算心率
		if(RR_intervals[7] != 0){
			int sum = 0;
			for(int i=0; i<8; i++){
				sum += RR_intervals[i];
			}
			sum /= 8;
			heart_rate = 60000/sum;
		}
	}
	
	/*
	 * 阈值初始化
	 */
	private int init_counter1;	//2秒计数器
	private int init_counter2;	//10组计数器
	private int max;
	private int[] maxes = new int[10];
	private void init_threshold(int data){
		if(data>max)
			max = data;
		//2秒到
		if(++init_counter1 == sample_rate*2){
			init_counter1 = 0;
			maxes[init_counter2] = max;	//将2秒内的最大值保存
			max = 0;	//最大值清零
			//10组到
			if(++init_counter2 == 10){
				init_counter2 = 0;
				Arrays.sort(maxes);	//对最大值排序
				//去首尾，求平均
				int sum = 0;
				for(int i=1; i<9; i++){
					sum += maxes[i];
				}
				sum /= 8;
				//设置阈值
				THRESHOLD_UP = sum/2;
				THRESHOLD_DOWN = sum/8;
				THRESHOLD_INITED = true;	//开启初始化完成标志位
				START = false; //防止直接计算第一个RR间期
			}
		}
	}
	
	/*
	 * 自适应调整阈值
	 */
	private int[] maxes_for_adjust = new int[4];
	private int maxes_counter;
	private void adjust_threshold(){
		maxes_for_adjust[maxes_counter] = max;
		max = 0;
		if(++maxes_counter == 4)
			maxes_counter = 0;
		//如果数组存满，计算平均值
		if(maxes_for_adjust[3] > 0){
			int sum = 0;
			for(int i=0; i<4; i++){
				sum += maxes_for_adjust[i];
			}
			sum /= 4;
			//更新阈值
			THRESHOLD_UP = sum/2;
			THRESHOLD_DOWN = sum/8;
		}
	}
	
	/*
	 * 总处理函数
	 */
	public void process(int[] ecg){
		N_buf = ecg.length;
		//输出数组
		int[] out = new int[N_buf];
		//逐点操作
		for(int i=0; i<N_buf; i++){
			//一阶差分，整流，平滑
			d1 = derivator1.get_derivative(ecg[i]);
			if(d1<0)
				d1 = -d1;
			d1 = smoother1.smooth(d1);
			//二阶差分，整流，平滑
			d2 = derivator2.get_derivative(d1);
			if(d2<0)
				d2 = -d2;
			d2 = smoother2.smooth(d2);
			//将一阶、二阶差分相加
			sum = (d1 + d2)*2;
			if(THRESHOLD_INITED){
				//阈值判断
				RR_counter++;
				if(QRS_state == 0){
					//上升沿阈值
					if(sum>THRESHOLD_UP){
						QRS_state = 1;
						if(START){	//只有当START开启才计算RR间期，否则第一个RR间期计算不正确
							//计算RR间期
							RR_interval = RR_counter*(1000/sample_rate);
							//计算心率
							calculate_heart_rate();
						}
						RR_counter = 0;
						START = true;	//开始START标志位
					}
				}
				else if(QRS_state == 1){
					QRS_counter++;
					//获取输出最大值，用以调整阈值
					if(sum>max)
						max = sum;
					//下降沿阈值
					if(sum<THRESHOLD_DOWN){
						QRS_state = 0;
						//计算QRS间期
						QRS_duration = QRS_counter*(1000/sample_rate);
						QRS_counter = 0;
						//心律失常
						arrhythmia.process(RR_interval, QRS_duration);
						display();
						//调整阈值
						adjust_threshold();
					}
				}
			}
			else{
				init_threshold(sum);
			}
			
			out[i] = sum;
		}
	}
	
	public class Arrhythmia{
		private List<Integer> RR_recent = new ArrayList<Integer>();	//最近8个RR间期
		private List<Integer> QRS_recent = new ArrayList<Integer>();	//最近8个QRS宽度
		private int RR_NORMAL = 0;	//RR间期边界
		private int RR_UP = 0;
		private int RR_DOWN = 0;
		private int QRS_NORMAL = 0;	//QRS宽度边界
		private int QRS_UP = 0;
		private int QRS_DOWN = 0;
		private boolean ABNORMAL = false;	//心律异常标志位
		private int abnormal_count = 0;		//计数器
		private boolean START = false;		//初始化完成标志
		
		private void process(int RR_interval, int QRS_duration)
		{
			if(!START)
				init(RR_interval, QRS_duration);	//初始化
			else{
				if( RR_interval < RR_UP && RR_interval > RR_DOWN){
					if(ABNORMAL){
						abnormal_count++;
						save();
					}
					ABNORMAL = false;
					RR_recent.add(RR_interval);
					RR_recent.remove(0);
					QRS_recent.add(QRS_duration);
					QRS_recent.remove(0);
					RR_NORMAL = mean(RR_recent);
					QRS_NORMAL = mean(QRS_recent);
					RR_UP = RR_NORMAL * 114 / 100;
					RR_DOWN = RR_NORMAL * 86 / 100;
					QRS_UP = QRS_NORMAL * 120 / 100;
					QRS_DOWN = QRS_NORMAL * 80 / 100;
				}
				else{
					ABNORMAL = true;
				}
			}
		}
		
		private int mean(List<Integer> list){
			int sum = 0;
			for(int i=0; i<8; i++){
				sum += list.get(i);
			}
			return sum/8;
		}
		
		private void init(int RR_interval, int QRS_duration){
			RR_recent.add(RR_interval);
			QRS_recent.add(QRS_duration);
			if(RR_recent.size() == 8){
				START = true;
				int RRsum = 0;
				int QRSsum = 0;
				for(int i=0; i<8; i++){
					RRsum += RR_recent.get(i);
					QRSsum += QRS_recent.get(i);
				}
				RR_NORMAL = RRsum / 8;
				RR_UP = RR_NORMAL * 114 / 100;
				RR_DOWN = RR_NORMAL * 86 / 100;
				QRS_NORMAL = QRSsum / 8;
				QRS_UP = QRS_NORMAL * 120 / 100;
				QRS_DOWN = QRS_NORMAL * 80 / 100;
			}
		}
		
		/*
		 * 通知WaveActivity保存
		 */
		private void save(){
			Message msg = WaveActivity.handler.obtainMessage();
			msg.what = 5;
			WaveActivity.handler.sendMessage(msg);
		}
		
	}
	
	

}