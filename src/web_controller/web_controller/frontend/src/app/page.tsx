import Controller from "@/comps/Controller"
import styles from "./page.module.css";

export default function Home() {
  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <h1 className={styles.title}>Hermes Controller</h1>
        <Controller />
      </main>
    </div>
  );
}
