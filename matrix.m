syms c1 c2 c3 c4 c5
syms s1 s2 s3 s4 s5
syms d1 a2 a3 a4 d5
syms c23 s23
syms c234 s234

% Ma trận A1
A1 = [c1, 0, s1, 0;
      s1, 0, -c1, 0;
      0, 1, 0, d1;
      0, 0, 0, 1];

% Ma trận A2
A2 = [c2, -s2, 0, a2*c2;
      s2, c2, 0, a2*s2;
      0, 0, 1, 0;
      0, 0, 0, 1];

% Ma trận A3
A3 = [c3, -s3, 0, a3*c3;
      s3, c3, 0, a3*s3;
      0, 0, 1, 0;
      0, 0, 0, 1];

% Ma trận A4
A4 = [c4, 0, s4, 0;
      s4, 0, -c4, 0;
      0, 1, 0, 0;
      0, 0, 0, 1];

% Ma trận A5
A5 = [c5, -s5, 0, 0;
      s5, c5, 0, 0;
      0, 0, 1, d5;
      0, 0, 0, 1];

A123 = [c1 * c23, -c1 * s23, s1, a2 * c1 * c2 + a3 * c1 * c23;
        s1 * c23, -s1 * s23, -c1, a2 * s1 * c2 + a3 * s1 * c23;
        s23, c23, 0, d1 + a2 * s2 + a3 * s23;
        0, 0, 0, 1];

A1234 = [c1 * c234, s1, c1 * s234, c1* (a2 * c2 + a3 * c23);
         s1 * c234, -c1, s1 * s234, s1* (a2 * c2 + a3 * c23);
         s234, 0, -c234, d1 + a2 * s2 + a3 * s23;
         0, 0, 0, 1];

c23 = c2 * c3 - s2 * s3;
s23 = s2 * c3 + c2 * s3;



% Tính toán ma trận đồng nhất T = A1 * A2 * A3 * A4 * A5
T123 = A1 * A2 * A3;
T = A4 * A5;

% Hiển thị kết quả
disp('Ma trận đồng nhất T123:');
disp(T123);

disp('Ma trận đồng nhất T:');
disp(T);