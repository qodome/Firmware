package com.example.wavebt_simulation;

import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;

import android.util.Log;

public class TcpClientThread extends Thread{
	public final String ip = "192.168.1.100";
	public final int port = 4000;
	public Socket socket = null;
	public OutputStream output = null;
	public int[] dataECG = null;
	public byte[] dataMOTION = null;
	/*
	 * 构造函数，连接服务器
	 */
	public TcpClientThread(){
       	Runnable runnable = new Runnable(){

				@Override
				public void run() {
					// TODO Auto-generated method stub
	                try {  
	                    System.out.println("Client：Connecting");  
	                    //IP地址和端口号（对应服务端）
	                    socket = new Socket(ip, port);
	                    output = socket.getOutputStream();
	                } catch (UnknownHostException e1) {  
	                    e1.printStackTrace();  
	                } catch (IOException e) {  
	                    e.printStackTrace();  
	                }  
	  
				}
        		
        	};
        	new Thread(runnable).start();
        	start();
	}
	/*
	 * 线程run方法
	 */
	public void run(){
		int seqNum = 0;
		int SampleRate = 100;
		while(true){
			synchronized(this){
				try {
					wait();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			if(dataECG != null){
				try{
				byte[] title = {'E','X','G','B','T'};
				output.write(title);
				output.write(0x02);
				
				int length = dataECG.length*2 + 6;
				byte l = (byte)length;
				byte h = (byte)(length>>8);
				output.write(l);
				output.write(h);
				l = (byte)seqNum;
				h = (byte)(seqNum>>8);
				output.write(l);
				output.write(h);
				seqNum++;
				output.write(0x00);
				output.write(0x00);
				l = (byte)SampleRate;
				h = (byte)(SampleRate>>8);
				output.write(l);
				output.write(h);
				for(int i=0; i<dataECG.length; i++){
					l = (byte)dataECG[i];
					h = (byte)(dataECG[i]>>8);
						output.write(l);
						output.write(h);
				}
				/*
				int length = dataECG.length;
				byte l = (byte)length;
				byte h = (byte)(length<<8);
				output.write(l);
				output.write(h);
				output.write(dataECG);
				*/
				}catch(IOException e){
					e.printStackTrace();
				}
				dataECG = null;
			}
			if(dataMOTION != null){
				try{
					byte[] title = {'M','O','T','I','O','N'};
					output.write(title);	//发送标签
					int length = dataMOTION.length;
					byte l = (byte)length;
					byte h = (byte)(length>>8);
					output.write(l);
					output.write(h);	//发送length
					output.write(dataMOTION);	//发送MOTION数据包
				}catch(IOException e){
					e.printStackTrace();
				}
				dataMOTION = null;
			}
		}
	}
	
	public void sendECG(int[] s){
		dataECG = new int[s.length];
		System.arraycopy(s, 0, dataECG, 0, s.length);
		synchronized(this){
			notifyAll();
		}
	}
	
	public void sendMOTION(byte[] s){
		dataMOTION = new byte[s.length];
		System.arraycopy(s, 0, dataMOTION, 0, s.length);
		synchronized(this){
			notifyAll();
		}
	}
}
