package com.example.wavebt_simulation;

public class Filter {
	private HighpassFilter highpass_filter = new HighpassFilter();
	private static final int OFFSET = 100;
	/*
	 * 高通滤波器类
	 */
	private class HighpassFilter{
		private int y1;
		private int[] x = new int[66];
		private int n = 32;
		
		private int highpass(int data){
			int y0;
			x[n] = x[n+33] = data;
			y0 = y1 + x[n] - x[n+32];
			y1 = y0;
			if(--n<0)
				n = 32;
			return(x[n+16] - (y0>>5));
		}
	}
	
	/*
	 *处理函数
	 */
	public int[] highpass(int[] data){
		int N_buf = data.length;
		int[] out = new int[N_buf];
		for(int i=0; i<N_buf; i++){
			out[i] = highpass_filter.highpass(data[i]) + OFFSET;//加上偏移量
		}
		return out;
	}
	
}