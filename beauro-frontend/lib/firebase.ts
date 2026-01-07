import { initializeApp } from "firebase/app";
import { getDatabase } from "firebase/database";

// ▼▼▼ 여기에 복사한 설정값을 붙여넣으세요 ▼▼▼
const firebaseConfig = {
  apiKey: "AIzaSyCQutCdckffzqAQcn7iuVjtBK_yV0Xe4O0",
  authDomain: "beauro-ac0ad.firebaseapp.com",
  databaseURL: "https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "beauro-ac0ad",
  storageBucket: "beauro-ac0ad.firebasestorage.app",
  messagingSenderId: "445574960836",
  appId: "1:445574960836:web:1a43282cd792eea2c85b91"
};

// ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

// Firebase 초기화
const app = initializeApp(firebaseConfig);

// DB 내보내기 (다른 파일에서 가져다 씀)
export const db = getDatabase(app);