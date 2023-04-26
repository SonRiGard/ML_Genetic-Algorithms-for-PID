**Điều chỉnh hệ số PID từ các phương pháp Ziegler–Nichols method và** 

**Genetic Algorithms (GA)**

# Cơ sở lí thuyết 
## Phương pháp Ziegler
Phương pháp Ziegler– là một phương pháp điều chỉnh bộ điều khiển PID được phát triển bởi John G. Ziegler và Nathaniel B. Nichols. Phương pháp này được thực hiện bằng cách thiết lập thông số độ lợi khâu I (tích phân) và khâu D (vi phân) về không (0,zero). Độ lợi khâu P (tỷ lệ, khuếch đại), độ lợi KP được tăng lên từ không (0) cho đến khi nó đạt đến độ lợi Ku tối đa, mà đầu ra của vòng điều khiển dao động với biên độ không đổi. Ku và chu kỳ dao Tu  được sử dụng để thiết lập độ lợi P, I, và D tùy thuộc vào loại điều khiển được sử dụng:

![](Aspose.Words.4d8b4093-d53f-457c-9a27-6026f0019b60.001.png)
## Phương pháp Genetic Algorithms (GA)
(Không nhằm mục tiêu trình bày về giải thuật GA. Nó chỉ được sử dụng như một công cụ để giải bài toán tối ưu, nhằm đạt được các giá trị {Kp\_opt, Kd\_ opt, Ki\_ opt} thỏa các hàm mục tiêu với không gian tìm kiếm được giới hạn bởi Phương pháp Ziegler)

Trước tiên, đối tượng điều khiển được xem xét như một hộp đen – điều này phù hợp với các ứng dụng trong thực tế. Một chu kỳ nhận dạng hành vi của hệ thống được xác lập, dựa vào đáp ứng bước vòng hở của đối tượng. Từ đáp ứng này, giải thuật Z-N được áp dụng để xác định ba thông số của bộ điều khiển PID. Ba thông số này là cơ sở để giới hạn không gian tìm kiếm của giải thuật GA. Nhiệm vụ của giải thuật GA là chọn lọc bộ ba {Kp, Kd, Ki} tối ưu cho bộ điều khiển PID, thỏa mãn một trong các hàm mục tiêu IAE, ITAE và MSE (Hình 1).

|![](Aspose.Words.4d8b4093-d53f-457c-9a27-6026f0019b60.002.png)|<p>yr(t): Tín hiệu tham khảo; </p><p>y(t): Đáp ứng của hệ thống; </p><p>u(t): Tín hiệu điều khiển </p><p>e(t): Sai biệt giữa tín hiệu tham khảo và đáp ứng của thệ thống.</p>|
| - | - |

**Hình 1: Mô hình tổng quát của hệ thống điều khiển**

Hàm mục tiêu

Trong hệ điều khiển vòng kín Hình 1, gọi e(t) là sai biệt giữa tín hiệu tham khảo yr(t) và tín hiệu đáp ứng y(t) của thệ thống, thì: 

e(t)= y<sub>r</sub>(t) - y(t) 							     

Các hàm mục tiêu của quá trình tinh chỉnh bộ điều khiển, trong bài toán này, được định nghĩa như sau: (*Tham khảo trong các tiêu chuẩn đánh giá tối ưu hóa đáp ứng quá độ )*

![](Aspose.Words.4d8b4093-d53f-457c-9a27-6026f0019b60.003.png)

Nhiệm vụ của giải thuật GA được áp dụng là tìm kiếm các giá trị {Kp\_opt, Kd\_ opt, Ki\_ opt} tối ưu của bộ điều khiển PID, mà ở đó các hàm J<sub>i</sub> (i=1,3) đạt giá trị cực tiểu. Nói cách khác, hàm mục tiêu của giải thuật GA là: 

min {Ji(1,2,3)} 

Nhằm giới hạn không gian tìm kiếm của giải thuật GA, ta giả thiết các giá trị tối ưu {Kp\_opt, Kd\_ opt, Ki\_ opt} nằm xung quanh giá trị {Kp\_Z-N, Kd\_ Z-N, Ki\_Z-N} đạt được từ giải thuật Z-N. Các giới hạn tìm kiếm tương ứng cho ba thông số của bộ điều khiển PID như sau:

![](Aspose.Words.4d8b4093-d53f-457c-9a27-6026f0019b60.004.png)

Trong đó, các hệ số α và β được chọn sao cho không gian tìm kiếm đủ rộng để chứa được giá trị tối ưu mong muốn.

![](Aspose.Words.4d8b4093-d53f-457c-9a27-6026f0019b60.005.png)

**Hình 2: Lưu đồ tiến trình GA xác định thông số bộ điều khiển PID**



