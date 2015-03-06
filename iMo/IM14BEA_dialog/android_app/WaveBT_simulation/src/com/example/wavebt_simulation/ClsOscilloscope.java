package com.example.wavebt_simulation;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.apache.http.NameValuePair;
import org.apache.http.message.BasicNameValuePair;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceView;

public class ClsOscilloscope {

	class FileLogger {
		private FileOutputStream exg_output = null;
		private FileInputStream exg_input = null;
		private File exg_folder;
		File current_file;
		private String exg_folder_name;
		private int current_unix_time = 0;
		protected List<Integer> next_ut_list = new ArrayList<Integer>();
		private int buffer_current_unix_time = 0;
		private int START_UNIX_TIME;//1400164035;
		 
		/*
		 * Initialize folder
		 */
		public FileLogger(String folderName, int TIME) {
			START_UNIX_TIME = TIME;
			exg_folder = new File(Environment.getExternalStorageDirectory() + "/" + folderName);
			if (!exg_folder.exists()) {
				exg_folder.mkdir();
			}
			exg_folder_name = new String(Environment.getExternalStorageDirectory() + "/" + folderName + "/");
		}
		
		public void Stop() {
			if (exg_output != null) {
			try {
				exg_output.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			}
		}
				
		public void ReaderInit() {
			current_unix_time = START_UNIX_TIME;
			do {
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} while (current_unix_time == 0);
			buffer_current_unix_time = current_unix_time;
			Log.i("EXG Wave", "Buffer Management open initial file: " + Integer.toString(buffer_current_unix_time));
			try {
				exg_input = new FileInputStream(new File(exg_folder_name + Integer.toString(buffer_current_unix_time)));
				byte tmpBuf[] = new byte[12];
				int bytesToRead = 12;
				int readRet = 0;
				
				while (bytesToRead > 0) {
					readRet = exg_input.read(tmpBuf, 0, bytesToRead);
					if (readRet < 0) {
						Log.e("EXG Wave", "Failed to read file");
					}
					bytesToRead -= readRet;
				}
			} catch (FileNotFoundException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}	
		}
		
		public int ReaderAvailable() throws IOException {
			return exg_input.available();
		}
		
		public int ReaderRead(byte[] buf, int offset, int length) throws IOException {
			return exg_input.read(buf, offset, length);
		}
		
		public void ReaderSwitchFile() {
			if (next_ut_list.size() > 0) {
				buffer_current_unix_time = next_ut_list.get(0);
				next_ut_list.remove(0);
				try {
					exg_input.close();
				} catch (IOException e2) {
					// TODO Auto-generated catch block
					e2.printStackTrace();
				}
				Log.i("EXG Wave", "Buffer switched to file " + Integer.toString(buffer_current_unix_time));
				try {
					exg_input = new FileInputStream(new File(exg_folder_name + Integer.toString(buffer_current_unix_time)));
					byte tmpBuf[] = new byte[12];
					int bytesToRead = 12;
					int readRet = 0;
					
					while (bytesToRead > 0) {
						readRet = exg_input.read(tmpBuf, 0, bytesToRead);
						if (readRet < 0) {
							Log.e("EXG Wave", "Failed to read file");
						}
						bytesToRead -= readRet;
					}
				} catch (FileNotFoundException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}	
		}
	};
	
	public int skip_buf_num = 1;
	public int skip_buf_num_delta = 1;
	
	public int start_y = 0;
	public int start_y_delta = 20;
	
	public int total_y = 1024;
	public int total_y_delta = 2;
	public int alive = 0;
	public int frames_per_sec = 5;
	public int total_write_count = 0;
	public int total_read_count = 0;
    public OutputStream outStream = null;
    private int START_TIME_ECG = 1406182222;
    private int START_TIME_MOTION = 1406527826;
    private FileLogger exgLogger;
    private FileLogger motionLogger;

    private Derivative derivative = new Derivative();
    private Filter filter = new Filter();
    public DrawThread drawThread = null;
    private TcpClientThread clientThread = new TcpClientThread();
    /*
    int send_over_socket = 0;
    Socket client = null;
    DataOutputStream dout;
	
    public void start_socket(String sock_info) {
    	String [] temp = null;
    	temp = sock_info.split(":");
    	SocketAddress addr = new InetSocketAddress(temp[0], Integer.parseInt(temp[1]));
    	
    	try {
    		Log.i("EXG Wave", "begin connect to: "+temp[0]+":"+temp[1]);
			client = new Socket();
			client.connect(addr, 2000);
			Log.i("EXG Wave", "Socket connected");
			dout = new DataOutputStream(client.getOutputStream());
			send_over_socket = 1;
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }
    */
    
	/**
	 * 初始化
	 */
	public void initOscilloscope() {
	}

	public void motion_toggle() {
		byte[] msg = new byte[6];
		/* 0x03 means motion rate toggle */
		msg[0] = 'C';
		msg[1] = 'M';
		msg[2] = 'D';		
		msg[3] = 0x03;
		msg[4] = 0x00;
		msg[5] = 0x00;
		try {
			outStream.write(msg);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	} 	
	
	public void set_trigger() {
		byte[] msg = new byte[6];
		/* 0x04 means trigger */
		msg[0] = 'C';
		msg[1] = 'M';
		msg[2] = 'D';		
		msg[3] = 0x04;
		msg[4] = 0x00;
		msg[5] = 0x00;
		try {
			outStream.write(msg);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	} 
	
	public void set_sample_rate(int rate) {
		byte[] msg = new byte[6];
		/* 0x05 means sample rate */
		msg[0] = 'C';
		msg[1] = 'M';
		msg[2] = 'D';		
		msg[3] = 0x05;
		msg[4] = (byte)(rate & 0xff);
		msg[5] = (byte)((rate >> 8) & 0xff);
		try {
			outStream.write(msg);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
	}
	
	public void adjust_speed(int delta) {
		if (delta == 1) {
			if (skip_buf_num < 5) {
				skip_buf_num += skip_buf_num_delta;
			}
		} else {
			if (skip_buf_num > 1) {
				skip_buf_num -= skip_buf_num_delta;
			}
		}
	}
	
	public void adjust_start_y(int delta) {
		if (delta == 1) {
			if (start_y < (1023-start_y_delta)) {
				start_y += start_y_delta;
			}
		} else {
			if (start_y > (start_y_delta)) {
				start_y -= start_y_delta;
			}
		}
	}
	
	public void adjust_total_y(int delta) {
		if (delta == 1) {
			total_y /= total_y_delta;
		} else {
			if (total_y > total_y_delta) {
				total_y *= total_y_delta;
			}
		}
	}
	
	/**
	 * 开始
	 * 
	 * @param recBufSize
	 *            AudioRecord的MinBufferSize
	 */
	public void Start(SurfaceView sfv, SurfaceView sfv_acc, Paint mPaint, int time_ecg, int time_motion) {
		START_TIME_ECG = time_ecg;
		START_TIME_MOTION = time_motion;
	    exgLogger = new FileLogger("EXG_DATA", START_TIME_ECG);
	    motionLogger = new FileLogger("MOTION_DATA", START_TIME_MOTION);  

		drawThread = new DrawThread(sfv, sfv_acc, mPaint);// 开始绘制线程
		drawThread.start();
		//Register with server
		List<NameValuePair> login_parm = new ArrayList<NameValuePair>();
		login_parm.add(new BasicNameValuePair("user", "user"));
		login_parm.add(new BasicNameValuePair("password", "USER"));
		/*
		try {
			String ret_msg = HttpUtils.post("http://dan.dminorstudio.com/mobile/user/login", 
					"{\"username\":\"user\", \"password\":\"USER\"}");
			Log.i("EXG_Wave", "login result: "+ret_msg);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		new PostData().start();
		*/
	}

	/**
	 * 停止
	 */
	public void Stop() {
		//关闭FileLogger
		exgLogger.Stop();
		motionLogger.Stop();
		//关闭DrawThread中使用的定时器
		drawThread.timer.cancel();
		drawThread.task.cancel();

		/*
		try {
			String ret_msg = HttpUtils.post("http://dan.dminorstudio.com/mobile/user/logout", "");
			Log.i("EXG_Wave", "logout result: "+ret_msg);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
	}
	
	public void switchfile_ecg(int next_unix_time){
		exgLogger.next_ut_list.add(next_unix_time);
		Log.d("BLE","Switch file success!");
	}
	
	public void switchfile_motion(int next_unix_time){
		motionLogger.next_ut_list.add(next_unix_time);
		Log.d("BLE","Switch file success!");
	}
	
	/*
	 * ECG的BufferManagement
	 */
	class BufferManagement {
		
		/* paint_list to keep track of historical buffers */
		private ArrayList<int[]> paint_list = new ArrayList<int[]>();
		
		/* number of points per buffer */
		private int per_buf_points = 0;
		
		/* 
		 * input buffer and its related state machine,
		 * to keep track of data fragments
		 */
		private byte[] head_buf = new byte[3];
		private int head_idx = 0;
		private byte[] data_buf;
		private int data_idx = 0;
		private int[] data_pool;
		
		private int screen_width = 0;
		/*
		 * FIXME: assume sample_per_sec is larger than screen_width!!!
		 */
		private int sample_per_sec = 0;		/* number of samples/sec */
		private int prev_sample_per_sec = 0;
		private int down_sample_rate = 0;	/* number of samples/shown sample */
		private int sub_sample_counter = 0;
		private int remain_points = 0;
		private int read_head_stage = 0;
		private short msg_length = 0;		
		/* the single temporary buffer */
		int[] tmp_buf = null;
		int tmp_i = 0;
		int tmp_buf_debt = 0;
		
		public BufferManagement(int width) {
			// We choose the latest header among all headers in EXG_DATA folder
			// First wait until EXG_DATA folder show up
			exgLogger.ReaderInit();
			screen_width = width;
		}
		
		public void fill_buffer(int buffer[]) {
			int i = 0;
			int paint_i = 0;
			int paint_buf_i = 0;
			int[] paint_buf = null;
			
			for (i = 0; i < buffer.length; i++) {
				buffer[i] = 0;
			}
			if (paint_list.isEmpty()) {
				//Log.i("EXG Wave", "fill buffer exit because NULL");
				return;
			}
			i = 0;
			paint_buf = paint_list.get(0);
			while (i < buffer.length) {
				if (paint_buf_i < per_buf_points) {
					buffer[i] = paint_buf[paint_buf_i];
					i++;
					paint_buf_i++;
				} else {
					paint_buf_i = 0;
					paint_i++;
					if (paint_list.size() > paint_i) {
						paint_buf = paint_list.get(paint_i);
					} else {
						return;
					}
				}
			}
		}
		
		public int get_show_buffer(int buffer[]) {	
			int remove_threshold = 0;
			
			//Log.i("EXG Wave", "get_show_buffer");
			if (per_buf_points == 0) {
				remove_threshold = 20;	//一个画板最多画20个数据包
			} else {
				remove_threshold = (screen_width/per_buf_points);	//一个画板画多少个数据包
			}
			if (paint_list.size() > remove_threshold) {
				//Log.i("EXG Wave", "remove head buffer");
				paint_list.remove(0);	//paint_list超出屏幕宽度，左移
			}
			if (remain_points == 0) {
				if (pull_more_data() != 0) {
					fill_buffer(buffer);
					return -1;
				}
			}
			tmp_buf_debt++;
			while (tmp_buf_debt > 0) {
				while (tmp_i < per_buf_points) {
					/* get one sample point from data_pool */
					while (sub_sample_counter != (down_sample_rate-1)) {
						if (remain_points > 0) {
							/* skip over data_idx */
							//Log.i("EXG Wave", "skip one");
							data_idx++;
							remain_points--;
							sub_sample_counter++;
							total_read_count++;
						} else {
							if (pull_more_data() != 0) {
								fill_buffer(buffer);
								return -1;
							}
						}
					}
					if (remain_points > 0) {
						//Log.i("EXG Wave", "sample: "+data_pool[data_idx]);
						tmp_buf[tmp_i] = data_pool[data_idx];
						tmp_i++;
						sub_sample_counter = 0;
						data_idx++;
						remain_points--;
						total_read_count++;
					} else {
						if (pull_more_data() != 0) {
							fill_buffer(buffer);
							return -1;
						}
					}
				}
				paint_list.add(paint_list.size(), tmp_buf);
				tmp_buf_debt--;
				tmp_i = 0;
				tmp_buf = new int[per_buf_points];
			}
			fill_buffer(buffer);
			return 0;
		}

		private int pull_more_data() {
			int rc = 0;
			
			//Log.i("EXG Wave", "pull_more_data");
			if (remain_points != 0) {
				Log.e("EXG Wave", "should not be called when still has data");
				return -1;
			}
			if (read_head_stage == 0) {
				//Log.i("EXG Wave", "read_head_stage == 0");
				head_idx = 0;
				data_idx = 0;
				try {
					if (exgLogger.ReaderAvailable() < 3) {
						//Log.i("EXG Wave", "header not available");
						//At this point, need to search for alternative file
						exgLogger.ReaderSwitchFile();
						return -1;
					}
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				while (head_idx < 3) {
					try {
						rc = exgLogger.ReaderRead(head_buf, head_idx, (3-head_idx));
						head_idx += rc;
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				if (head_buf[0] != 0x02) {
					Log.e("EXG Wave", "Error in exg.data: header type ne 0x02");
				}

				msg_length = (short) ((head_buf[1] & 0xff) | ((head_buf[2] & 0xff) << 8));
				data_buf = new byte[msg_length];
				read_head_stage = 1;
			}
			
			/* wait until there is at least msg_length data to read */
			try {
				if (exgLogger.ReaderAvailable() < msg_length) {
					//Log.i("EXG Wave", "msg_length not available");
					return -1;
				}
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			while (data_idx < msg_length) {
				try {
					rc = exgLogger.ReaderRead(data_buf, data_idx, (msg_length-data_idx));
					data_idx += rc;
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			/* Now we have data, add them to paint_list */
			//short channel_num;
			//short seq_num;
			//seq_num = (short) ((data_buf[0] & 0xff) | ((data_buf[1] & 0xff) << 8));
			//channel_num = (short) ((data_buf[2] & 0xff) | ((data_buf[3] & 0xff) << 8));
			sample_per_sec = (short) ((data_buf[4] & 0xff) | ((data_buf[5] & 0xff) << 8));
			remain_points = (msg_length-6)/2;
			read_head_stage = 0;
			data_pool = new int[remain_points];
			for (int i = 0; i < remain_points; i++) {
				data_pool[i] = (int) ((data_buf[6+i*2] & 0xff) | ((data_buf[6+i*2+1] & 0xff) << 8))/32;
			}
			//差分法寻找QRS波群
			derivative.process(data_pool);
			//高通滤波消除基线漂移
			data_pool = filter.highpass(data_pool);
			//TCP发送数据到服务器
			//clientThread.sendECG(data_pool);
			
			if (prev_sample_per_sec != sample_per_sec) {
				/* 
				 * sample/sec changed, this could be the init case, 
				 * or user changed sample rate
				 */
				prev_sample_per_sec = sample_per_sec;
				paint_list.clear();
				sub_sample_counter = 0;
				tmp_i = 0;
				tmp_buf_debt = 0;
				if (sample_per_sec >= screen_width) {
					/* down sampling */
					down_sample_rate = sample_per_sec/screen_width;
					per_buf_points = (sample_per_sec/down_sample_rate)/frames_per_sec;
					
				} else {
					/* full sample */
					down_sample_rate = 1;
					per_buf_points = sample_per_sec/frames_per_sec;
					if (per_buf_points < 1) {
						per_buf_points = 1;
					}
				}
				tmp_buf = new int[per_buf_points];
				data_idx = 0;
				return -1;
			} 
			data_idx = 0;	
			return 0;
		}
	}
	
	/*
	 * MOTION的BufferManagement
	 */
	class BufferManagement_Acc {
		
		/* paint_list to keep track of historical buffers */
		private ArrayList<int[]> paint_list = new ArrayList<int[]>();
		
		/* number of points per buffer */
		private int per_buf_points = 0;
		
		/* 
		 * input buffer and its related state machine,
		 * to keep track of data fragments
		 */
		private byte[] head_buf = new byte[2];
		private int head_idx = 0;
		private byte[] data_buf;
		private int data_idx = 0;
		private int[] data_pool;	//加速度向量模
		private byte[] data_pool_x;	//x轴加速度
		private byte[] data_pool_y;	//y轴加速度
		private byte[] data_pool_z;	//z轴加速度
		
		private int screen_width = 0;
		/*
		 * FIXME: assume sample_per_sec is larger than screen_width!!!
		 */
		private int sample_per_sec = 0;		/* number of samples/sec */
		private int prev_sample_per_sec = 0;
		private int down_sample_rate = 0;	/* number of samples/shown sample */
		private int sub_sample_counter = 0;
		private int remain_points = 0;
		private int read_head_stage = 0;
		private short msg_length = 0;		
		/* the single temporary buffer */
		int[] tmp_buf = null;
		int tmp_i = 0;
		int tmp_buf_debt = 0;
		
		public BufferManagement_Acc(int width) {
			// We choose the latest header among all headers in EXG_DATA folder
			// First wait until EXG_DATA folder show up
			motionLogger.ReaderInit();
			screen_width = width;
		}
		
		public void fill_buffer(int buffer[]) {
			int i = 0;
			int paint_i = 0;
			int paint_buf_i = 0;
			int[] paint_buf = null;
			
			for (i = 0; i < buffer.length; i++) {
				buffer[i] = 0;
			}
			if (paint_list.isEmpty()) {
				//Log.i("EXG Wave", "fill buffer exit because NULL");
				return;
			}
			i = 0;
			paint_buf = paint_list.get(0);
			while (i < buffer.length) {
				if (paint_buf_i < per_buf_points) {
					buffer[i] = paint_buf[paint_buf_i];
					i++;
					paint_buf_i++;
				} else {
					paint_buf_i = 0;
					paint_i++;
					if (paint_list.size() > paint_i) {
						paint_buf = paint_list.get(paint_i);
					} else {
						return;
					}
				}
			}
		}
		
		public int get_show_buffer(int buffer[]) {	
			int remove_threshold = 0;
			
			//Log.i("EXG Wave", "get_show_buffer");
			if (per_buf_points == 0) {
				remove_threshold = 20;	//一个画板最多画20个数据包
			} else {
				remove_threshold = (screen_width/per_buf_points);	//一个画板画多少个数据包
			}
			if (paint_list.size() > remove_threshold) {
				//Log.i("EXG Wave", "remove head buffer");
				paint_list.remove(0);	//paint_list超出屏幕宽度，左移
			}
			if (remain_points == 0) {
				if (pull_more_data() != 0) {
					fill_buffer(buffer);
					return -1;
				}
			}
			tmp_buf_debt++;
			while (tmp_buf_debt > 0) {
				while (tmp_i < per_buf_points) {
					/* get one sample point from data_pool */
					while (sub_sample_counter != (down_sample_rate-1)) {
						if (remain_points > 0) {
							/* skip over data_idx */
							//Log.i("EXG Wave", "skip one");
							data_idx++;
							remain_points--;
							sub_sample_counter++;
							total_read_count++;
						} else {
							if (pull_more_data() != 0) {
								fill_buffer(buffer);
								return -1;
							}
						}
					}
					if (remain_points > 0) {
						//Log.i("EXG Wave", "sample: "+data_pool[data_idx]);
						tmp_buf[tmp_i] = data_pool[data_idx];
						tmp_i++;
						sub_sample_counter = 0;
						data_idx++;
						remain_points--;
						total_read_count++;
					} else {
						if (pull_more_data() != 0) {
							fill_buffer(buffer);
							return -1;
						}
					}
				}
				paint_list.add(paint_list.size(), tmp_buf);
				tmp_buf_debt--;
				tmp_i = 0;
				tmp_buf = new int[per_buf_points];
			}
			fill_buffer(buffer);
			return 0;
		}

		private int pull_more_data() {
			int rc = 0;
			
			//Log.i("EXG Wave", "pull_more_data");
			if (remain_points != 0) {
				Log.e("EXG Wave", "should not be called when still has data");
				return -1;
			}
			if (read_head_stage == 0) {
				//Log.i("EXG Wave", "read_head_stage == 0");
				head_idx = 0;
				data_idx = 0;
				try {
					if (motionLogger.ReaderAvailable() < 2) {
						//Log.i("EXG Wave", "header not available");
						//At this point, need to search for alternative file
						motionLogger.ReaderSwitchFile();
						return -1;
					}
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				while (head_idx < 2) {
					try {
						rc = motionLogger.ReaderRead(head_buf, head_idx, (2-head_idx));
						head_idx += rc;
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				
				msg_length = (short) ((head_buf[0] & 0xff) | ((head_buf[1] & 0xff) << 8));
				data_buf = new byte[msg_length];
				read_head_stage = 1;
			}
			
			/* wait until there is at least msg_length data to read */
			try {
				if (motionLogger.ReaderAvailable() < msg_length) {
					//Log.i("EXG Wave", "msg_length not available");
					return -1;
				}
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			while (data_idx < msg_length) {
				try {
					rc = motionLogger.ReaderRead(data_buf, data_idx, (msg_length-data_idx));
					data_idx += rc;
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			//clientThread.sendMOTION(data_buf);	//发送加速度数据到服务器端
			/* Now we have data, add them to paint_list */
			//short channel_num;
			//short seq_num;
			//seq_num = (short) ((data_buf[0] & 0xff) | ((data_buf[1] & 0xff) << 8));
			//channel_num = (short) ((data_buf[2] & 0xff) | ((data_buf[3] & 0xff) << 8));
			sample_per_sec = (short) ((data_buf[2] & 0xff) | ((data_buf[3] & 0xff) << 8));
			remain_points = (msg_length-4)/3;
			read_head_stage = 0;
			data_pool = new int[remain_points];
			data_pool_x = new byte[remain_points];
			data_pool_y = new byte[remain_points];
			data_pool_z = new byte[remain_points];
			/*读取加速度数据x,y,z，计算向量模*/
			for (int i = 0; i < remain_points; i++) {
				data_pool_x[i] = data_buf[4+i*3];
				data_pool_y[i] = data_buf[4+i*3+1];
				data_pool_z[i] = data_buf[4+i*3+2];
				int temp = data_pool_x[i]*data_pool_x[i] + data_pool_y[i]*data_pool_y[i] + data_pool_z[i]+ data_pool_z[i];
				//data_pool[i] = (int) Math.sqrt(temp);
				data_pool[i] = temp;
			}
			if (prev_sample_per_sec != sample_per_sec) {
				/* 
				 * sample/sec changed, this could be the init case, 
				 * or user changed sample rate
				 */
				prev_sample_per_sec = sample_per_sec;
				paint_list.clear();
				sub_sample_counter = 0;
				tmp_i = 0;
				tmp_buf_debt = 0;
				if (sample_per_sec >= screen_width) {
					/* down sampling */
					down_sample_rate = sample_per_sec/screen_width;
					per_buf_points = (sample_per_sec/down_sample_rate)/frames_per_sec;
					
				} else {
					/* full sample */
					down_sample_rate = 1;
					per_buf_points = sample_per_sec/frames_per_sec;
					if (per_buf_points < 1) {
						per_buf_points = 1;
					}
				}
				tmp_buf = new int[per_buf_points];
				data_idx = 0;
				return -1;
			} 
			data_idx = 0;	
			return 0;
		}
	}
	
	/**
	 * 负责绘制inBuf中的数据
	 */
	private int zoom_rate = 1;
	protected boolean PAUSE = false; //屏幕冻结
	protected int[] paint_buffer_pause = null;
	class DrawThread extends Thread {
		private SurfaceView sfv;// ECG画板
		private Paint mPaint;// 画笔
		private int max_y = 0;
		private int max_x = 0;
		Timer timer = new Timer();
		public int [] paint_buffer = null;
		private BufferManagement buf_mgmt = null;
		private int rc;
		private int debug_i = 0;
		private SurfaceView sfv_acc;//Acc画板
		private int max_y_acc = 0;
		private int max_x_acc = 0;
		private BufferManagement_Acc buf_mgmt_acc = null;
		private int[] paint_buffer_acc = null;
		

		
	    TimerTask task = new TimerTask(){
	    	/* 
	    	 * Each run period shows a frame of data
	    	 */
	        public void run() {
	        	debug_i++;
	        	if (debug_i == frames_per_sec) {
	        		debug_i = 0;
	        		//Log.i("EXG Wave", "total_write_count: "+total_write_count+" total_read_count: "+total_read_count);
	        	}
	        	/* 初始化ECG画布 */
	        	if (max_y == 0) {
	        		max_y = sfv.getHeight();
	        		max_x = sfv.getWidth();
	        		paint_buffer = new int[max_x];
	        		buf_mgmt = new BufferManagement(max_x);
	        	}
	        	rc = buf_mgmt.get_show_buffer(paint_buffer);
	        	if (rc == -1) {
	        		//Log.i("EXG Wave", "get_show_buffer returned error");
	        	}
	        	/*初始化ACC画布*/
	        	if(max_y_acc == 0){
	        		max_y_acc = sfv_acc.getHeight();
	        		max_x_acc = sfv_acc.getWidth();
	        		paint_buffer_acc = new int[max_x_acc];
	        		buf_mgmt_acc = new BufferManagement_Acc(max_x_acc);
	        	}
	        	rc = buf_mgmt_acc.get_show_buffer(paint_buffer_acc);
	        	/*判断是否冻结*/
	        	if(PAUSE == true && paint_buffer_pause == null){
	        		paint_buffer_pause = new int[paint_buffer.length];
	        		System.arraycopy(paint_buffer, 0, paint_buffer_pause, 0, paint_buffer.length);
	        		//保存指定波形段
					//save(paint_buffer_pause);
	        	}
	        	else if(PAUSE == false)
	        		paint_buffer_pause = null;
	        	//绘制波形
	        	//！！这里必须接收异常，否则当退出程序时，onDestroy()还没有执行，timer仍在工作，
	        	//而SimpleDraw()中要使用SurfaceView，就会抛出空指针异常。不catch会死机。
	        	try{
		        	SimpleDraw(paint_buffer);
		        	SimpleDraw_Acc(paint_buffer_acc);
	        	}catch(Exception e){
	        		e.printStackTrace();
	        	}
	        }  	          
	    }; 

	    TimerTask task_acc = new TimerTask(){

			@Override
			public void run() {
				// TODO Auto-generated method stub
	        	/*初始化ACC画布*/
	        	if(max_y_acc == 0){
	        		max_y_acc = sfv_acc.getHeight();
	        		max_x_acc = sfv_acc.getWidth();
	        		paint_buffer_acc = new int[max_x_acc];
	        		buf_mgmt_acc = new BufferManagement_Acc(max_x_acc);
	        	}
	        	rc = buf_mgmt_acc.get_show_buffer(paint_buffer_acc);
	        	try{
		        	//SimpleDraw(paint_buffer);
		        	SimpleDraw_Acc(paint_buffer_acc);
	        	}catch(Exception e){
	        		e.printStackTrace();
	        	}
			}
	    	
	    };
	    
		public DrawThread(SurfaceView sfv, SurfaceView sfv_acc, Paint mPaint) {
			this.sfv = sfv;
			this.sfv_acc = sfv_acc;
			this.mPaint = mPaint;
			timer.schedule(task, 1000, 1000/frames_per_sec); 
		}

		public void run() {			
			/* Everything runs in timer handler */
		}

		/**
		 * 绘制指定区域
		 * 
		 * @param start
		 *            X轴开始的位置(全屏)
		 * @param buffer
		 *            缓冲区
		 * @param rate
		 *            Y轴数据缩小的比例
		 * @param baseLine
		 *            Y轴基线
		 */
		void SimpleDraw(int[] buffer) {
			Canvas canvas = sfv.getHolder().lockCanvas(
					new Rect(0, 0, sfv.getWidth(), sfv.getHeight()));// 关键:获取画布
			canvas.drawColor(Color.BLACK);// 清除背景
			//Log.i("EXG Wave", "create "+sfv.getWidth()+" X "+sfv.getHeight());
			int y;
			int oldX = 0, oldY = 0;
			if(!PAUSE){
				for (int i = 0; i < buffer.length/zoom_rate; i++) {// 有多少画多少
					int x = i*zoom_rate;
					if ((buffer[i]/* + total_y/2*/) < start_y) {
						buffer[i] = start_y;
					} else if (buffer[i] > (start_y+total_y)) {
						buffer[i] = (start_y+total_y);
					}
					y = ((buffer[i] - start_y) * max_y) / total_y;
					y = max_y - y /*- max_y/2*/;
					if (oldX == 0) {
						oldY = y;
					}
					canvas.drawLine(oldX, oldY, x, y, mPaint);
					oldX = x;
					oldY = y;
				}
			}
			else{
				for (int i = 0; i < paint_buffer_pause.length/zoom_rate; i++) {// 有多少画多少
					int x = i*zoom_rate;
					if ((paint_buffer_pause[i]/* + total_y/2*/) < start_y) {
						paint_buffer_pause[i] = start_y;
					} else if (paint_buffer_pause[i] > (start_y+total_y)) {
						paint_buffer_pause[i] = (start_y+total_y);
					}
					y = ((paint_buffer_pause[i] - start_y) * max_y) / total_y;
					y = max_y - y /*- max_y/2*/;
					if (oldX == 0) {
						oldY = y;
					}
					canvas.drawLine(oldX, oldY, x, y, mPaint);
					oldX = x;
					oldY = y;
				}
			}
			sfv.getHolder().unlockCanvasAndPost(canvas);// 解锁画布，提交画好的图像z
		}
		
		/**
		 * 绘制加速度
		 * @param buffer 绘制的信号，即加速度向量模
		 */
		void SimpleDraw_Acc(int[] buffer){
			Canvas canvas = sfv_acc.getHolder().lockCanvas(
					new Rect(0, 0, sfv_acc.getWidth(), sfv_acc.getHeight()));// 关键:获取画布
			canvas.drawColor(Color.BLACK);// 清除背景
			int x;
			int y;
			int oldX = 0, oldY = 0;	
			for(int i=0; i<buffer.length; i++){
				x = i;
				//y = max_y_acc - (buffer[i] * max_y_acc)/256;
				y = max_y_acc - (buffer[i] * max_y_acc)/32768;

				if (oldX == 0) {
					oldY = y;
				}
				canvas.drawLine(oldX, oldY, x, y, mPaint);
				oldX = x;
				oldY = y;				
			}
			sfv_acc.getHolder().unlockCanvasAndPost(canvas);// 解锁画布，提交画好的图像z
		}
	}
	
	/*
	 * 将指定的波形段保存到SD卡
	 */
	public void save(int[] data, String file_name){
		/*建立文件夹*/
		File save_folder = new File(Environment.getExternalStorageDirectory()+"/EXG_DATA/save");
		if(!save_folder.exists())
			save_folder.mkdir();
		/*建立文件*/
		File file = new File(Environment.getExternalStorageDirectory()+"/EXG_DATA/save/"+file_name);
		if(!file.exists())
			try {
				file.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		/*保存数据*/
		FileOutputStream output = null;
		try {
			output = new FileOutputStream(file);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		byte[] buffer = new byte[data.length*2];
		int j = 0;
		for(int i=0; i<data.length; i++){
			buffer[j++] = (byte)data[i];
			buffer[j++] = (byte)(data[i]>>8);
		}
		try {
			output.write(buffer);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			output.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	/*
	 * 读取指定文件名的波形
	 */
	public void load(String file_name){
		/*寻找文件*/
		File file = new File(Environment.getExternalStorageDirectory()+"/EXG_DATA/save/"+file_name);
		if(!file.exists())
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		/*读取数据*/
		FileInputStream input = null;
		try{
			input = new FileInputStream(file);
		} catch(IOException e){
			e.printStackTrace();
		}
		byte[] buffer = new byte[(int) file.length()];
		try {
			input.read(buffer);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		int[] data = new int[buffer.length/2];
		for(int i=0; i<data.length; i++){
			data[i] = (int) ((buffer[i*2] & 0xff) | ((buffer[i*2+1] & 0xff) << 8));
		}
		paint_buffer_pause = data;
	}
	
	/*
	 * 保存心律失常波形
	 */
	public void save_arrhythmia(){
		final int[] data = new int[drawThread.paint_buffer.length];
		System.arraycopy(drawThread.paint_buffer, 0, data, 0, drawThread.paint_buffer.length);
		Runnable runnable = new Runnable(){

			@Override
			public void run() {
				/*建立文件夹*/
				File save_folder = new File(Environment.getExternalStorageDirectory()+"/EXG_DATA/Arrhythmia");
				if(!save_folder.exists())
					save_folder.mkdir();
				/*建立文件*/
				int file_name = (int) (System.currentTimeMillis() / 1000L);
				File file = new File(Environment.getExternalStorageDirectory()+"/EXG_DATA/Arrhythmia/"+file_name);
				if(!file.exists())
					try {
						file.createNewFile();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				/*保存数据*/
				FileOutputStream output = null;
				try {
					output = new FileOutputStream(file);
				} catch (FileNotFoundException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				byte[] buffer = new byte[data.length*2];
				int j = 0;
				for(int i=0; i<data.length; i++){
					buffer[j++] = (byte)data[i];
					buffer[j++] = (byte)(data[i]>>8);
				}
				try {
					
					output.write(buffer);
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				try {
					output.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
		};
		new Thread(runnable).start();
	}
	
}
