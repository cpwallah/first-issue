/*
    Problem Link: https://leetcode.com/problems/integer-to-roman/
*/

class Solution {
public:
    string intToRoman(int num) {
        string result;
        string symbols[13] = {"M", "CM", "D", "CD", "C", "XC", "L", "XL", "X", "IX", "V", "IV", "I"};
        int val[] = {1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1};

        for(int i = 0; num != 0; i++) {
            while(num >= val[i]) {
                num -= val[i];
                result += symbols[i];
            }
        }
        return result;
    }
};
/*
    Problem Link: https://leetcode.com/problems/longest-palindromic-substring/
*/

class Solution {
public:
    void expandBothSides(string &s, int left, int right, int &start_idx, int &max_len, const int n) {
        // expand the palindrome on both sides if there is a match
        while(left >= 0 && right < n && s[left] == s[right]) {
            // keep the max len palindromic substring
            if(right - left + 1 > max_len) {
                start_idx = left;
                max_len = right - left + 1;
            }
            // expand the window
            left--;
            right++;
        }
    }

    string longestPalindrome(string &s) {
        const int n = (int)s.size();
        int start_idx = 0; // starting index where the substring starts
        int max_len = 1; // single char is considered to be a substring
        int left, right; // 2 pointers

        for(int i = 1; i < n; i++) {
            // consecutive pointers _ _
            left = i - 1;
            right = i;
            expandBothSides(s, left, right, start_idx, max_len, n);

            // pointers with 1 gap in between _x_
            left = i - 1;
            right = i + 1;
            expandBothSides(s, left, right, start_idx, max_len, n);
        }
        return s.substr(start_idx, max_len);
    }
};

/*
    Problem Link: https://leetcode.com/problems/reverse-words-in-a-string/
*/

class Solution {
public:
    string reverseWords(string s) {
        stack<string> st;
        string word;
        int n = s.size();
        for(int i = 0; i < n; i++) {
            // leading spaces at the start
            if(word.empty()) {
                while(s[i] == ' ')
                    i++;
            }
            // add the characters to the string word
            if(s[i] != ' ')
                word += s[i];
            // push the words when space is encountered or i == (n - 1), for the last word
            if(s[i] == ' ' || i == n - 1) {
                if(!word.empty())
                    st.push(word);
                // reset string to store the next word
                word.clear();
            }
        }
        word.clear();
        while(!st.empty()) {
            word += st.top();
            st.pop();
            word += " ";
        }
        // remove the last added space 
        word.pop_back();
        return word;
    }
};

/*
    Problem Link: https://leetcode.com/problems/roman-to-integer/
*/

class Solution {
    unordered_map<char, int> m = {
        {'I', 1}, {'V', 5}, {'X', 10}, {'L', 50}, 
        {'C', 100}, {'D', 500}, {'M', 1000}
    };

public:
    int romanToInt(string &s) {
        int result = 0, n = s.size();
        int curr = m[s[0]];
        for(int i = 0; i < n; i++) {
            // next will be 0 if we reach the end of the string
            int next = m[s[i + 1]];
            if(curr < next)
                result -= curr;
            else
                result += curr;
            curr = next;
        }
        return result;
    }
};
